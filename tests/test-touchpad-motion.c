/*
 * test-touchpad-motion.c — Unit tests for touchpad absolute-to-relative conversion
 *
 * Mirrors the touchpad helpers in scripts/wiggle-monitor.c and checks:
 *   - Single-finger drags produce scaled relative deltas
 *   - A new contact never produces a jump from the previous contact
 *   - Two-finger scrolling and gestures produce no motion
 *   - Dropping from two fingers to one does not jump to the other finger
 *   - Device units are normalized by resolution, with a range fallback
 *
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <linux/input.h>

#define TOUCHPAD_COUNTS_PER_MM  40.0
#define TOUCHPAD_FALLBACK_MM    100.0

typedef struct {
    double scale_x;
    double scale_y;
    int    x;
    int    y;
    int    last_x;
    int    last_y;
    bool   has_last;
    bool   touching;
    int    fingers;
} TouchpadState;

static double abs_units_per_mm(const struct input_absinfo *abs) {
    if (abs->resolution > 0) return abs->resolution;
    double range = abs->maximum - abs->minimum;
    return range > 0 ? range / TOUCHPAD_FALLBACK_MM : 1.0;
}

static void touchpad_handle_event(TouchpadState *tp, const struct input_event *ev) {
    if (ev->type == EV_ABS) {
        if (ev->code == ABS_X) tp->x = ev->value;
        else if (ev->code == ABS_Y) tp->y = ev->value;
        return;
    }

    switch (ev->code) {
    case BTN_TOUCH:          tp->touching = ev->value != 0; break;
    case BTN_TOOL_FINGER:    if (ev->value) tp->fingers = 1; break;
    case BTN_TOOL_DOUBLETAP: if (ev->value) tp->fingers = 2; break;
    case BTN_TOOL_TRIPLETAP: if (ev->value) tp->fingers = 3; break;
    case BTN_TOOL_QUADTAP:   if (ev->value) tp->fingers = 4; break;
    case BTN_TOOL_QUINTTAP:  if (ev->value) tp->fingers = 5; break;
    default: break;
    }
}

static void touchpad_finish_frame(TouchpadState *tp, int *dx, int *dy) {
    bool tracking = tp->touching && tp->fingers == 1;

    if (tracking && tp->has_last) {
        *dx += (int)lround((tp->x - tp->last_x) * tp->scale_x);
        *dy += (int)lround((tp->y - tp->last_y) * tp->scale_y);
    }

    if (!tp->touching) tp->fingers = 0;
    tp->last_x = tp->x;
    tp->last_y = tp->y;
    tp->has_last = tracking;
}

/* ─── Test Harness ─── */

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name, expr) do { \
    tests_run++; \
    if (expr) { \
        printf("  ✓ %s\n", name); \
        tests_passed++; \
    } else { \
        printf("  ✗ %s (FAILED)\n", name); \
    } \
} while(0)

static void emit(TouchpadState *tp, unsigned short type, unsigned short code, int value) {
    struct input_event ev = { .type = type, .code = code, .value = value };
    touchpad_handle_event(tp, &ev);
}

// Feeds one evdev frame the way the kernel reports it and returns its delta.
static void frame(TouchpadState *tp, int fingers, int x, int y, int *dx, int *dy) {
    static const unsigned short tools[] = {
        BTN_TOOL_FINGER, BTN_TOOL_DOUBLETAP, BTN_TOOL_TRIPLETAP,
        BTN_TOOL_QUADTAP, BTN_TOOL_QUINTTAP,
    };

    emit(tp, EV_ABS, ABS_X, x);
    emit(tp, EV_ABS, ABS_Y, y);
    emit(tp, EV_KEY, BTN_TOUCH, fingers > 0);
    for (int i = 0; i < 5; i++)
        emit(tp, EV_KEY, tools[i], fingers == i + 1);

    *dx = 0;
    *dy = 0;
    touchpad_finish_frame(tp, dx, dy);
}

static TouchpadState make_touchpad(double scale) {
    TouchpadState tp;
    memset(&tp, 0, sizeof(tp));
    tp.scale_x = scale;
    tp.scale_y = scale;
    return tp;
}

/* ─── Tests ─── */

static void test_single_finger_drag(void) {
    printf("Single-finger drag:\n");
    TouchpadState tp = make_touchpad(1.0);
    int dx, dy, sum_x = 0, sum_y = 0;

    for (int i = 0; i <= 10; i++) {
        frame(&tp, 1, 1000 + i * 10, 500 - i * 5, &dx, &dy);
        sum_x += dx;
        sum_y += dy;
    }
    TEST("Total delta matches finger travel", sum_x == 100 && sum_y == -50);
}

static void test_new_contact_does_not_jump(void) {
    printf("New contacts:\n");
    TouchpadState tp = make_touchpad(1.0);
    int dx, dy;

    frame(&tp, 1, 2000, 2000, &dx, &dy);
    TEST("First contact produces no delta", dx == 0 && dy == 0);

    frame(&tp, 0, 2000, 2000, &dx, &dy);
    frame(&tp, 1, 100, 100, &dx, &dy);
    TEST("Re-placing a finger elsewhere produces no delta", dx == 0 && dy == 0);

    frame(&tp, 1, 110, 100, &dx, &dy);
    TEST("Motion after re-placing is tracked", dx == 10 && dy == 0);
}

static void test_multi_finger_ignored(void) {
    printf("Multi-finger gestures:\n");
    TouchpadState tp = make_touchpad(1.0);
    int dx, dy, moved = 0;

    for (int i = 0; i < 20; i++) {
        frame(&tp, 2, 1000, 1000 + i * 30, &dx, &dy);
        moved |= dx | dy;
    }
    TEST("Two-finger scroll produces no motion", moved == 0);

    for (int i = 0; i < 20; i++) {
        frame(&tp, 3, 1000 + i * 30, 1000, &dx, &dy);
        moved |= dx | dy;
    }
    TEST("Three-finger swipe produces no motion", moved == 0);
}

static void test_finger_count_drop(void) {
    printf("Finger count changes:\n");
    TouchpadState tp = make_touchpad(1.0);
    int dx, dy;

    frame(&tp, 2, 1000, 1000, &dx, &dy);
    frame(&tp, 1, 3000, 1000, &dx, &dy);
    TEST("Dropping to one finger does not jump", dx == 0 && dy == 0);

    frame(&tp, 1, 3020, 1000, &dx, &dy);
    TEST("Remaining finger is tracked", dx == 20 && dy == 0);
}

static void test_resolution_scaling(void) {
    printf("Resolution scaling:\n");
    struct input_absinfo with_res = { .minimum = 0, .maximum = 3000, .resolution = 20 };
    struct input_absinfo without_res = { .minimum = 0, .maximum = 3000, .resolution = 0 };

    TEST("Reported resolution is used", abs_units_per_mm(&with_res) == 20.0);
    TEST("Missing resolution falls back to range",
         fabs(abs_units_per_mm(&without_res) - 30.0) < 1e-9);

    TouchpadState tp = make_touchpad(TOUCHPAD_COUNTS_PER_MM / abs_units_per_mm(&with_res));
    int dx, dy;
    frame(&tp, 1, 1000, 1000, &dx, &dy);
    frame(&tp, 1, 1020, 1000, &dx, &dy);
    TEST("1 mm of finger travel maps to 40 counts", dx == 40 && dy == 0);
}

int main(void) {
    printf("Wiggle Touchpad Motion Tests\n");
    printf("============================\n\n");

    test_single_finger_drag();
    test_new_contact_does_not_jump();
    test_multi_finger_ignored();
    test_finger_count_drop();
    test_resolution_scaling();

    printf("\n============================\n");
    printf("Results: %d/%d tests passed\n", tests_passed, tests_run);

    return (tests_passed == tests_run) ? 0 : 1;
}
