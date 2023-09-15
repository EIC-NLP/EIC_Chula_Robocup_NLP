

from . import usb_pixel_ring_v1
from . import usb_pixel_ring_v2
from .apa102_pixel_ring import PixelRing

pixel_ring = usb_pixel_ring_v2.find()

if not pixel_ring:
    pixel_ring = usb_pixel_ring_v1.find()

if not pixel_ring:
    pixel_ring = PixelRing()


USAGE = '''
If the hardware is ReSpeaker 4 Mic Array for Pi or ReSpeaker V2,
there is a power-enable pin which should be enabled at first.
+ ReSpeaker 4 Mic Array for Pi:

    from gpiozero import LED
    power = LED(5)
    power.on()

+ ReSpeaker V2:

    import mraa
    power = mraa.Gpio(12)
    power.dir(mraa.DIR_OUT)
    power.write(0)
'''

def main():
    import time

    if isinstance(pixel_ring, usb_pixel_ring_v2.PixelRing):
        print('Found ReSpeaker USB 4 Mic Array')
    elif isinstance(pixel_ring, usb_pixel_ring_v1.UsbPixelRing):
        print('Found ReSpeaker USB 6+1 Mic Array')
    else:
        print('Control APA102 RGB LEDs via SPI')
        print(USAGE)

    pixel_ring.think()
    time.sleep(3)
    pixel_ring.off()
    time.sleep(1)


if __name__ == '__main__':
    main()

