/*
 * GPIO Rotary Coder
 *
 * Copyright (c) 2009 Filip Navara
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "sysbus.h"
#include "console.h"

typedef struct RotaryCoderState {
    SysBusDevice busdev;
    qemu_irq out[2];
    uint8_t state;
    uint8_t key_left;
    uint8_t key_right;
    uint8_t key_left_alt;
    uint8_t key_right_alt;
    uint8_t extension;
} RotaryCoderState;

static void rotary_update(RotaryCoderState *s, int direction)
{
    s->state += direction;
    s->state %= 4;

    qemu_set_irq(s->out[0], s->state == 1 || s->state == 2);
    qemu_set_irq(s->out[1], s->state == 2 || s->state == 3);
}

static void rotary_keyboard_event(void *opaque, int keycode)
{
    RotaryCoderState *s = opaque;

    if (keycode == 0xe0 && !s->extension) {
        s->extension = 0x80;
        return;
    }

    if (!(keycode & 0x80)) {
        keycode &= 0x7f;
        keycode |= s->extension;

        if (keycode == s->key_left || keycode == s->key_left_alt) {
            rotary_update(s, 3);
        } else if (keycode == s->key_right || keycode == s->key_right_alt) {
            rotary_update(s, 1);
        }
    }

    s->extension = 0;
}

static const VMStateDescription vmstate_rotary = {
    .name = "gpio,rotary",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(state, RotaryCoderState),
        VMSTATE_UINT8(extension, RotaryCoderState),
        VMSTATE_END_OF_LIST()
    }
};


static int rotary_init(SysBusDevice *dev)
{
    RotaryCoderState *s = FROM_SYSBUS(RotaryCoderState, dev);

    qdev_init_gpio_out(&dev->qdev, s->out, 2);
    qemu_add_kbd_event_handler(rotary_keyboard_event, s);

    return 0;
}

static Property rotary_properties[] = {
    DEFINE_PROP_UINT8("key-left", RotaryCoderState, key_left, 0xcb),
    DEFINE_PROP_UINT8("key-right", RotaryCoderState, key_right, 0xcd),
    DEFINE_PROP_UINT8("key-left-alt", RotaryCoderState, key_left_alt, 0x4b),
    DEFINE_PROP_UINT8("key-right-alt", RotaryCoderState, key_left, 0x4d),
    DEFINE_PROP_END_OF_LIST(),
};

static void rotary_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = rotary_init;
    dc->props = rotary_properties;
    dc->vmsd = &vmstate_rotary;
}

static TypeInfo rotary_info = {
    .name  = "gpio,rotary",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(RotaryCoderState),
    .class_init = rotary_class_init,
};

static void rotary_register_types(void)
{
    type_register_static(&rotary_info);
}

type_init(rotary_register_types)
