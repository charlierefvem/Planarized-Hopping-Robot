import pyb, micropython, struct
from pyb import CAN, Pin
from encoder import Encoder
from time import ticks_us, ticks_add, ticks_diff

# Help screen
def printHelp():
    print('+---------+------------------------------------+')
    print('| Command | Description                        |')
    print('+---------+------------------------------------+')
    print('|    P    | Print the current encoder position |')
    print('|    V    | Print the current encoder velocity |')
    print('|    Z    | Zero the encoder                   |')
    print('|    H    | Print this menu again              |')
    print('|    C    | CAN INFO                           |')
    print('|    D    | Deinitialize CAN                   |') 
    print('|    R    | Reinitialize CAN                   |')   
    print('|    S    | Send CAN message                   |') 
    print('+---------+------------------------------------+')
    print()

# Message received callback
def cb0(bus, reason):
    global messageReceived
    messageReceived = True

if __name__ == '__main__':

    # Set state addresses
    S0_INIT = micropython.const(1)
    S1_UPD  = micropython.const(2)
    S2_ZERO = micropython.const(3)

    # Start in initialization state
    state = S0_INIT

    # Set timing scheme
    period = 2_500
    start_time = ticks_us()
    next_time = ticks_add(start_time, period)

    # Set up character keyboard port
    serport = pyb.USB_VCP()

    # Set up callback variable
    messageReceived = False

    # Set up data receiving variable
    idx = 0

    # Initialize everything
    # Set up encoder object
    encoder_1 = Encoder(Pin.cpu.B4, Pin.cpu.B5, 3, 10000)

    # Set up CAN pins
    RxPin = Pin(Pin.cpu.B8, mode = Pin.AF_PP, alt = Pin.AF9_CAN1)
    TxPin = Pin(Pin.cpu.B9, mode = Pin.AF_PP, alt = Pin.AF9_CAN1)

    # Set up CAN communication object
    can = CAN(1, CAN.NORMAL, baudrate=1_000_000, sjw = 1, bs1 = 12, bs2 = 2, auto_restart = True) # Make CAN object
    can.setfilter(0, CAN.LIST16, 0, (1, 2, 3, 4))  # Set a filter to receive messages with id = 1, 2, 3, and 4
    can.rxcallback(0, cb0) # Set up interrupt when a message comes in

    # Set up dataBytes object
    dataBytes = struct.pack('<ih', 0, 0)

    while True:

        # If a ping has been received, instantly do the task it asks for
        if messageReceived:
            while can.any(0):
                data = can.recv(0)
                messageReceived = False
                if data[0] == 3:
                    can.send(dataBytes, 1)
                    idx += 1

        else:

            # Check if time to run
            current_time = ticks_us()
            if ticks_diff(current_time, next_time) >= 0:
                
                # Set next time
                next_time = ticks_add(next_time, period)

                # Initialization state
                if state == S0_INIT:

                    # Go to next state and give user command list
                    printHelp()
                    state = S1_UPD

                # Constant updating state, check for pings
                elif state == S1_UPD:

                    # Are there any user commands?
                    if serport.any():

                        # What is the command?
                        charIn = serport.read(1).decode()

                        # Execute the command if recognized.
                        if charIn in {'p','P'}:
                            print(f'The current position is: {positionDec}')
                            print(f'The current velocity is: {velocityDec:.2f}')
                            print()
                        
                        elif charIn in {'v','V'}:
                            print(f'The current velocity is: {velocityDec:.2f}')
                            print()

                        elif charIn in {'z','Z'}:
                            print(f'Zeroing encoder...')
                            state = S2_ZERO

                        elif charIn in {'h','H'}:
                            printHelp()

                        elif charIn in {'c','C'}:
                            print(f'INFO: {can.info()}')
                            print(f'STATE: {can.state()}')
                            print()   

                        elif charIn in {'r','R'}:
                            can.init(CAN.NORMAL, baudrate=1_000_000, sjw = 1, bs1 = 12, bs2 = 2, auto_restart = True)
                            print('Reinitialized.')
                            print()

                        elif charIn in {'d','D'}:
                            can.deinit()
                            print('Deinitialized.')

                        elif charIn in {'s','S'}:
                            can.send(dataBytes, 1)

                        else:
                            print(f'{charIn} is not a recognized command.')
                            print()

                    # If no messages, just update/filter
                    else:
                        encoder_1.update()
                        positionDec = encoder_1.get_position()
                        velocityDec = encoder_1.get_delta()
                        dataBytes = struct.pack('<ih', round(positionDec), round(velocityDec*1000)) # pack position to a 4 byte int and velocity * 100 to a 2 byte int, 


                # Zero state
                elif state == S2_ZERO:
                    
                    # Zero encoder
                    encoder_1.zero()

                    # Tell user and go back to update state
                    print('Encoder zeroed.')
                    print()
                    state = S1_UPD
                
                # Just in case no state is set somehow
                else:
                    state = S1_UPD

            # If not time, then pass
            else:
                pass
