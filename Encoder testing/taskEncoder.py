'''!@file           taskEncoder.py
    @brief          A file to interact with and retrieve values from the encoder.
    @details        This file sets up a task to perform all data collection from 
                    the encoder. It operates on three states, one of which 
                    initializes the encoder to be set up. The other two are to 
                    constantly update the encoder and write data to shared 
                    variables or to zero the position value of the encoder. It 
                    operates on a set period and will set an internal timer 
                    that tells it when to run again based on the period. For a
                    more comprehensive state break down please read the following
                    state transition diagram:
                        
                    @image html Lab2EncoderStates.JPG
                    
                    As always, feel free to reference the source code at
                    https://bitbucket.org/seanwahl2023/me_305_labs/src/master/Lab%202/ 
                    however especially in this case since function variables
                    are not captured by Doxygen.
    @author         Sean Wahl
    @author         Grant Gabrielson
    @date           February 2, 2022
'''

import pyb, micropython, encoder
from time import ticks_us, ticks_add, ticks_diff

def taskEncoderFunction(taskName, period, zFlag, position, delta):
    '''!@brief              Routinely retrieves data from an encoder on a set period.
        @details            This generator function, given the right set of
                            inputs, is able to constantly update the encoder's
                            position and write these values to shared variables
                            that are accessible from other functions. If a user
                            inputs the right key, a flag can be raised to zero
                            the position and then the task will go back updating
                            the encoder values.
                            
        @param taskName     The arbitrary, chosen name of the task.
        @param period       The time interval at which this task should operate.
        @param zFlag        A shared variable that means the encoder should be zeroed.
        @param position     A shared variable that holds the current position value.
        @param delta        A shared variable that holds the last delta value.
    '''
    
    ## @brief Represents the initialization state of the task.
    #
    S0_INIT = micropython.const(0)
    
    ## @brief Represents the updating state of the task.
    #
    S1_UPD = micropython.const(1)
    
    ## @brief Represents the zeroing state of the task.
    #
    S2_ZERO = micropython.const(2)
    
    ## @brief       Denotes the state that the task is currently running in.
    #  @details     taskEncoder has three possible states. State 0 initializes
    #               the encoder and always sends the task to state 1. In state
    #               1, the encoder is constantly updating its position and 
    #               delta values and writing them to shared variables. This is
    #               so the values can be accessed from outside of this file as
    #               long as they were set up properly using the share class.
    #               State 2 gets triggered by a user input and will zero the
    #               encoder's position and then go back to state 1.
    #
    state = S0_INIT
    
    ## @brief The initial time at which the task has first started running.
    #
    start_time = ticks_us()
    last_time = start_time
    
    ## @brief       The next time the task should run.
    #  @details     By using the set period input, next_time is a calculated
    #               value of when the task should run again in order to be
    #               operating at said period.
    #
    next_time = ticks_add(start_time, period)

    th_hat = 0
    thd_hat = 0
    alpha = 0.85
    beta = 0.1

    
    while True:
        ## @brief The current time when the function tries to run again.
        #
        current_time = ticks_us()
        if ticks_diff(current_time, next_time) >= 0:    
            
            next_time = ticks_add(next_time, period)
            Ts = (current_time - last_time)/1_000_000
            last_time = current_time
            
            # Set up encoder object
            if state == S0_INIT:
                ## @brief The encoder object that is set up using the encoder class.
                #
                encoder_1 = encoder.Encoder(pyb.Pin.cpu.C6, pyb.Pin.cpu.C7, 8, 360)
                state = S1_UPD
                yield None
            
            # Constantly update the position and delta values
            elif state == S1_UPD:
                if zFlag.read():
                    state = S2_ZERO
                    yield None
                    
                else:
                    encoder_1.update()
                    #th_r = encoder_1.get_position()
                    #th_hat = th_hat + Ts*thd_hat
                    #inn = th_r - th_hat
                    #th_hat = th_hat + alpha*inn
                    #thd_hat = thd_hat + (beta/Ts)*inn

                    #position.write(th_hat)
                    #delta.write(thd_hat)
                    position.write(encoder_1.get_position())
                    delta.write(encoder_1.get_delta())
                    
                    yield None
            
            # Set encoder position to zero
            elif state == S2_ZERO:
                encoder_1.zero()
                th_hat = 0
                thd_hat = 0
                state = S1_UPD
                yield zFlag.write(False)
            
            # Just in case
            else:
                print('something went wrong')
                break
        else:
            yield None