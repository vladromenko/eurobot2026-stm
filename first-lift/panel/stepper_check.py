"""Read both L6474 drivers, without enabling a motor."""
from robot_panel.protocol import make_packet, read_response
from robot_panel.command_codes import BAUD_RATE, CMD_LIFT2_GET_STATE
import serial
import sys

port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
with serial.Serial(port, BAUD_RATE, timeout=1, write_timeout=1, exclusive=True) as link:
    link.reset_input_buffer()
    link.write(make_packet(CMD_LIFT2_GET_STATE, bytes([1])))
    response = read_response(link, 2)
data = response.data
if response.command != CMD_LIFT2_GET_STATE or len(data) != 13 or data[0] != 0xD2:
    sys.exit("Ответ другой версии прошивки")
for channel, start in ((1, 1), (2, 7)):
    tval, mode, trip, off = data[start:start + 4]
    status = int.from_bytes(data[start + 4:start + 6], "little")
    print(f"Stepper {channel}: TVAL=0x{tval:02X}, STEP_MODE={mode & 7}, OCD_TH=0x{trip:02X}, "
          f"TOFF_MIN=0x{off:02X}, STATUS=0x{status:04X}, HiZ={status & 1}")
    if channel == 2 and (status in (0, 0xFFFF) or (status & 0x1E01) != 0x1E01
                         or tval != 0x0F or mode & 7 != 4 or trip != 8):
        sys.exit("СТОП: канал 2 не подтвердил штатные параметры в покое.")
print("Канал 2 готов к одиночной проверке. Выход выключен.")
