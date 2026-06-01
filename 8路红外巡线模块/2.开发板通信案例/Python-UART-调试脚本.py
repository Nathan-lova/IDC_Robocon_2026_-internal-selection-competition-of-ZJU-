import serial

SERIAL_PORT = "COM3"  # 修改为实际串口号
BAUD = 115200


def parse_packet(pkt: bytes):
    return [((pkt[i] << 8) | pkt[i + 1]) for i in range(0, 16, 2)]


def main():
    with serial.Serial(SERIAL_PORT, BAUD, timeout=1) as ser:
        ser.write(b"DS")  # 切数字模式，可改 b"AD"
        while True:
            pkt = ser.read(16)
            if len(pkt) != 16:
                continue
            print(parse_packet(pkt))


if __name__ == "__main__":
    main()

