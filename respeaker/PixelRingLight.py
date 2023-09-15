from tuning import Tuning
import usb.core
import usb.util
import time
from flask import Flask
from flask_apscheduler import APScheduler
import logging

TIMEOUT = 8000


def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return

    # configuration = dev.get_active_configuration()

    # interface_number = None
    # for interface in configuration:
    #     interface_number = interface.bInterfaceNumber

    #     if dev.is_kernel_driver_active(interface_number):
    #         dev.detach_kernel_driver(interface_number)

    return dev


dev = find()
Mic_tuning = Tuning(dev)

app = Flask(__name__)
scheduler = APScheduler()
# set max instances of the scheduler to 2


def trace():
    write(0)


def mono(color):
    write(1, [(color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, 0])


def set_color(rgb=None, r=0, g=0, b=0):
    if rgb:
        mono(rgb)
    else:
        write(1, [r, g, b, 0])


def off():
    mono(0)


def listen(direction=None):
    write(2)


wakeup = listen


def speak():
    write(3)


def think():
    write(4)


wait = think


def spin():
    write(5)


def show(data):
    write(6, data)


customize = show


def set_brightness(brightness):
    write(0x20, [brightness])


def set_color_palette(a, b):
    write(0x21, [(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF, 0,
                 (b >> 16) & 0xFF, (b >> 8) & 0xFF, b & 0xFF, 0])


def set_vad_led(state):
    write(0x22, [state])


def set_volume(volume):
    write(0x23, [volume])


def change_pattern(pattern=None):
    print('Not support to change pattern')


def write(cmd, data=[0]):
    dev.ctrl_transfer(
        usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR
        | usb.util.CTRL_RECIPIENT_DEVICE, 0, cmd, 0x1C, data, TIMEOUT)


def close():
    """
    close the interface
    """
    usb.util.dispose_resources(dev)


def better_spin():
    '''
    more noticable spinning
    1 loop
    '''
    r, g, b = 0, 0, 0
    data = [r, g, b, 0] * 12
    for i in range(12):
        data[i * 4 + 0] = 0
        data[i * 4 + 1] = 255
        data[i * 4 + 2] = 0
        data[i * 4 + 3] = 0
        show(data)
        time.sleep(0.05)
        data[i * 4 + 0] = 0
        data[i * 4 + 1] = 0
        data[i * 4 + 2] = 0
        data[i * 4 + 3] = 0


global isSpinning
isSpinning = False


def scheduleTask():
    if isSpinning:
        # print('Spinning')
        better_spin()
    else:
        # print('Not spinning')
        listen()


@app.route('/spin')
def startSpinLights():
    global isSpinning
    isSpinning = True
    return 'Spinning'


@app.route('/stop')
def stopSpinLights():
    global isSpinning
    isSpinning = False
    return 'Stopped'


@app.route('/direction')
def getDirectionOfVoice():
    return {'direction': Mic_tuning.direction}


if __name__ == '__main__':
    scheduler.add_job(id='Scheduled Task',
                      func=scheduleTask,
                      trigger='interval',
                      seconds=0.05)
    scheduler.start()
    # use logging to add filter for apscheduler messages that starts with Execution of job
    logging.basicConfig()
    logging.getLogger('apscheduler.scheduler').setLevel(logging.CRITICAL)

    app.run(host='0.0.0.0', port=5000)
