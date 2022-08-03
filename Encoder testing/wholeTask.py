import pyb, micropython
from pyb import CAN
from encoder import Encoder
from time import ticks_us, ticks_add, ticks_diff

if __name__ == '__main__':
    
    # Set state addresses
    S0_INIT = micropython.const(1)
    S1_UPD  = micropython.const(2)
    S2_SEND = micropython.const(3)
    S3_ZERO = micropython.const(4)

    # Start in initialization state
    state = S0_INIT

    # Set timing periods
    period = 10,000
    start_time = ticks_us()
    next_time = ticks_add(start_time, period)

    # Set alpha beta filter constants
    #alpha = 0.85
    #beta  = 0.005
    #last_time = start_time

    while True:
        # Check if time to run
        current_time = ticks_us()
        if ticks_diff(current_time, next_time) >= 0:
            
            # Set next time
            next_time = ticks_add(next_time, period)

            # Find time period taken to get here
            #Ts = ticks_diff(current_time, last_time)/1_000_000
            #last_time = current_time

            # Initialization state
            if state == S0_INIT:

                # Set up encoder object
                encoder_1 = Encoder(pyb.Pin.cpu.C6, pyb.Pin.cpu.C7, 8, 360)

                # Set up CAN communication object
                can = CAN(1, CAN.NORMAL, baudrate = 1_000_000)
                posID = 0x0000
                velID = 0xFFF3
                can.setfilter(0, CAN.LIST32, 0, (posID, velID))
                FIFO = 0

                # Instantiate alpha beta values
                #th_hat = 0
                #thd_hat = 0

                # Go to next state
                state = S1_UPD

                yield None

            # Constant updating state, check for pings
            elif state == S1_UPD:

                # Are there any CAN messages?
                if can.any(FIFO):

                    # What is the CAN message?
                    msg = can.recv(FIFO)

                    # If this message, then go to zero state
                    if msg == 10:
                        state = S3_ZERO
                        yield None

                    # If this message, then go to send state
                    elif msg == 20:
                        state = S2_SEND
                        yield None

                    # If non-standard message, just update as normal
                    else:
                        encoder_1.update()

                        #th_r = encoder_1.get_position()
                        #th_hat = th_hat + Ts*thd_hat
                        #inn = th_r - th_hat
                        #th_hat = th_hat + alpha*inn
                        #thd_hat = thd_hat + (beta/Ts)*inn
                        yield None

                # If no messages, just update
                else:
                    encoder_1.update()

                    #th_r = encoder_1.get_position()
                    #th_hat = th_hat + Ts*thd_hat
                    #inn = th_r - th_hat
                    #th_hat = th_hat + alpha*inn
                    #thd_hat = thd_hat + (beta/Ts)*inn
                    yield None

            # Update and send state
            elif state == S2_SEND:

                # Update position and velocity as normal
                encoder_1.update()
                
                #th_r = encoder_1.get_position()
                #th_hat = th_hat + Ts*thd_hat
                #inn = th_r - th_hat
                #th_hat = th_hat + alpha*inn
                #thd_hat = thd_hat + (beta/Ts)*inn
                
                # Send data through CAN channels
                can.send(encoder_1.get_position(), posID)
                can.send(encoder_1.get_delta(), velID)

                # Go back to update state
                state = S1_UPD
                yield None

            # Zero state
            elif state == S3_ZERO:
                
                # Zero encoder and reset ab filter
                encoder_1.zero()
                #th_hat = 0
                #thd_hat = 0

                # Go back to update state
                state = S1_UPD
                yield None
            
            # Just in case no state is set somehow
            else:
                state = S1_UPD
                yield None

        # If not time, then pass
        else:
            yield None