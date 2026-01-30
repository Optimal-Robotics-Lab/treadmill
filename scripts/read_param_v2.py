import sys
import serial.tools.list_ports  # New requirement: pip install pyserial
from pymodbus.client import ModbusSerialClient


def find_serial_port():
    """Dynamically finds the best candidate for a serial port."""
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        return None

    # Priority 1: Look for common USB-Serial adapter keywords
    for p in ports:
        # Check description/hardware ID for common chips like CH340 or FTDI
        if any(
            key in p.description.upper()
            for key in ["USB", "SERIAL", "CH340", "CP210", "FTDI"]
        ):
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device

    # Priority 2: Just return the first available port if no match
    print(f"No specific match found. Defaulting to: {ports[0].device}")
    return ports[0].device


def read_inverter_param(target):
    address = int(target)
    port = find_serial_port()

    if not port:
        print("Error: No serial ports detected on this system.")
        return

    # Setup Modbus Connection
    client = ModbusSerialClient(
        port=port, baudrate=9600, parity="E", stopbits=1, bytesize=8, timeout=1
    )

    if not client.connect():
        print(f"Failed to connect on {port}. Check your permissions or cable.")
        return

    # Read Register (Slave ID 1 is the Weg CFW-11 factory default)
    result = client.read_holding_registers(address=address, count=1, slave=1)

    if not result.isError():
        value = result.registers[0]
        print("--- CFW-11 Read Success ---")
        print(f"Register Address: {address}")
        print(f"Raw Value:        {value}")
    else:
        print(f"Modbus Error: {result}")

    client.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python read_param.py [param_number]")
    else:
        read_inverter_param(sys.argv[1])
