# TODO write script to read a parameter from the cfw11 inverter
import sys
import glob
from pymodbus.client import ModbusSerialClient


def find_serial_port():
    """Attempts to find the USB adapter automatically."""
    for path in glob.glob("/dev/serial/by-id/*"):
        if any(x in path for x in ["1a86", "QinHeng", "USB_Single_Serial"]):
            return path
    # Fallback for Windows or direct Linux paths
    return "/dev/ttyUSB0"


def read_inverter_param(target):
    # 1. Determine the Register Address
    address = int(target)

    # 2. Setup Modbus Connection (matching CFW-11 defaults from your snippet)
    port = find_serial_port()
    client = ModbusSerialClient(
        port=port, baudrate=9600, parity="E", stopbits=1, bytesize=8, timeout=1
    )

    if not client.connect():
        print(f"Failed to connect on {port}")
        return

    # 3. Read Register
    # Slave ID is typically 1 for these setups
    result = client.read_holding_registers(address=address, count=1, slave=1)

    if not result.isError():
        value = result.registers[0]
        print("--- CFW-11 Read Success ---")
        # print(f"Parameter: {name}")
        print(f"Register:  {address}")
        print(f"Raw Value: {value}")
    else:
        print(f"Error reading parameter {address}: {result}")

    client.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python read_param.py [param_number]")
        print("Example: python read_param.py 100")
        print("For a list of parameters, refer to the CFW-11 programming manual.")
    else:
        read_inverter_param(sys.argv[1])
