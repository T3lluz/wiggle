// Exercise the production state recovery, including contacts already down at startup.
#define main monitor_main
#include "../scripts/wiggle-monitor.c"
#undef main
#include <assert.h>

int main(void) {
    struct libevdev *dev = libevdev_new();
    assert(dev);
    struct input_absinfo axis = { .minimum = 0, .maximum = 4000, .resolution = 40 };
    assert(libevdev_enable_event_code(dev, EV_ABS, ABS_X, &axis) == 0);
    assert(libevdev_enable_event_code(dev, EV_ABS, ABS_Y, &axis) == 0);
    const unsigned int keys[] = { BTN_TOUCH, BTN_TOOL_FINGER, BTN_TOOL_DOUBLETAP };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
        assert(libevdev_enable_event_code(dev, EV_KEY, keys[i], NULL) == 0);
    libevdev_set_event_value(dev, EV_KEY, BTN_TOUCH, 1);
    libevdev_set_event_value(dev, EV_KEY, BTN_TOOL_DOUBLETAP, 1);
    TouchpadState tp;
    touchpad_init(&tp, dev);
    assert(tp.touching && tp.fingers == 2 && !tp.has_last);

    // libevdev has recovered the new state after a dropped two-to-one transition.
    tp.has_last = true;
    libevdev_set_event_value(dev, EV_KEY, BTN_TOOL_DOUBLETAP, 0);
    libevdev_set_event_value(dev, EV_KEY, BTN_TOOL_FINGER, 1);
    libevdev_set_event_value(dev, EV_ABS, ABS_X, 2000);
    touchpad_sync_state(&tp, dev);
    assert(tp.touching && tp.fingers == 1 && tp.x == 2000 && !tp.has_last);
    int dx = 0, dy = 0;
    touchpad_finish_frame(&tp, &dx, &dy);
    assert(dx == 0 && dy == 0);
    tp.x += 80;
    touchpad_finish_frame(&tp, &dx, &dy);
    assert(dx == 80 && dy == 0);

    libevdev_set_event_value(dev, EV_KEY, BTN_TOUCH, 0);
    libevdev_set_event_value(dev, EV_KEY, BTN_TOOL_FINGER, 0);
    touchpad_sync_state(&tp, dev);
    assert(!tp.touching && tp.fingers == 0 && !tp.has_last);
    libevdev_free(dev);
    puts("Touchpad startup and dropped-event recovery passed");
    return 0;
}
