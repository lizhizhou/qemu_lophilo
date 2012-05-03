/*
 * GPIO 4x4 Matrix Keyboard
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

typedef struct KeyPadState {
    SysBusDevice busdev;
    qemu_irq out[8];
    uint32_t in[8];
    void *keys;
    uint16_t keymask;
    uint16_t keymap[256];
    uint8_t extension;
} KeyPadState;

static void keypad_update(KeyPadState *s)
{
    int pin;
    int keymask;

    for (pin = 0; pin < 4; pin++) {
        if (s->keymask & (0xf << (pin * 4))) {
            keymask = s->keymask >> (pin * 4);
            qemu_set_irq(
                s->out[pin + 4],
                (s->in[0] && (keymask & 0x1)) ||
                (s->in[1] && (keymask & 0x2)) ||
                (s->in[2] && (keymask & 0x4)) ||
                (s->in[3] && (keymask & 0x8)));
        } else {
            qemu_set_irq(s->out[pin + 4], -1);
        }

        if (s->keymask & (0x1111 << pin)) {
            keymask = s->keymask >> pin;
            qemu_set_irq(
                s->out[pin],
                (s->in[4] && (keymask & 0x1)) ||
                (s->in[5] && (keymask & 0x10)) ||
                (s->in[6] && (keymask & 0x100)) ||
                (s->in[7] && (keymask & 0x1000)));
        } else {
            qemu_set_irq(s->out[pin], -1);
        }
    }
}

static void keypad_set_pin(void *opaque, int pin, int level)
{
    KeyPadState *s = opaque;

    s->in[pin] = level;
    keypad_update(s);
}

static void keypad_keyboard_event(void *opaque, int keycode)
{
    KeyPadState *s = opaque;
    int index;

    if (keycode == 0xe0 && !s->extension) {
        s->extension = 0x80;
        return;
    }

    index = (keycode & 0x7f) | s->extension;
    s->extension = 0;
    if (keycode & 0x80)
        s->keymask &= ~s->keymap[index];
    else if (keycode)
        s->keymask |= s->keymap[index];

    keypad_update(s);
}

static const VMStateDescription vmstate_keypad = {
    .name = "gpio,keypad",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(in, KeyPadState, 8),
        VMSTATE_UINT16(keymask, KeyPadState),
        VMSTATE_UINT8(extension, KeyPadState),
        VMSTATE_END_OF_LIST()
    }
};

static int keypad_init(SysBusDevice *dev)
{
    KeyPadState *s = FROM_SYSBUS(KeyPadState, dev);
    int i;
    uint32* keys = s->keys;

    for (i = 0; i < 16; i++) {
        s->keymap[keys[i] & 0xff] = 1 << i;
    }

    qdev_init_gpio_in(&dev->qdev, keypad_set_pin, 8);
    qdev_init_gpio_out(&dev->qdev, s->out, 8);
    qemu_add_kbd_event_handler(keypad_keyboard_event, s);
    return 0;
}

static Property keypad_properties[] = {
    DEFINE_PROP_PTR("keys", KeyPadState, keys),
    DEFINE_PROP_END_OF_LIST(),
};

static void keypad_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = keypad_init;
    dc->props = keypad_properties;
    dc->vmsd = &vmstate_keypad;
}

static TypeInfo keypad_info = {
    .name  = "gpio,keypad",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(KeyPadState),
    .class_init    = keypad_class_init,
};

static void keypad_register_types(void)
{
    type_register_static(&keypad_info);
}

type_init(keypad_register_types)
