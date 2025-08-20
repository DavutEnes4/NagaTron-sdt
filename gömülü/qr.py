from evdev import InputDevice, categorize, ecodes

device_path = '/dev/sdt_qrscanner'  # udev ile atanmış sabit yol

try:
    device = InputDevice(device_path)
    device.grab()
except FileNotFoundError:
    print(f"'{device_path}' cihazı bulunamadı! Cihaz bağlı mı ve udev kuralı doğru mu?")
    exit(1)

print(f"Listening for QR input on {device_path}")

qr_data = ''

for event in device.read_loop():
    if event.type == ecodes.EV_KEY:
        key_event = categorize(event)
        if key_event.keystate == key_event.key_down:
            key = key_event.keycode

            if isinstance(key, list):
                key = key[0]

            if key == 'KEY_ENTER':
                print(f"QR Code scanned: {qr_data}")
                qr_data = ''
            elif key.startswith('KEY_'):
                char = key[4:].lower()
                if len(char) == 1:
                    qr_data += char
