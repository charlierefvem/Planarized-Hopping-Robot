'''!@file           taskUser.py
    @brief          A file to take user input and communicate with taskEncoder.
    @details        taskUser sets up a generator function to run on a specified
                    period. This function is used to take in user input and
                    grab values from taskEncoder through shared variables as
                    needed. If taskEncoder handles all encoder relations, taskUser
                    handles all user relations. taskUser changes states based on
                    the user input and otherwise will wait for a command. For a
                    breakdown of the states, please check out the function details.
                    For a more comprehensive understanding of the state transitions,
                    reference the following diagram:
                        
                    @image html Lab2UserStates.JPG
                    
                    As always, feel free to reference the source code at
                    https://bitbucket.org/seanwahl2023/me_305_labs/src/master/Lab%202/ 
                    however especially in this case since function variables
                    are not captured by Doxygen.
    @author         Sean Wahl
    @author         Grant Gabrielson
    @date           February 2, 2022
'''

from time import ticks_us, ticks_add, ticks_diff
import pyb, micropython, array

def taskUserFunction(taskName, period, zFlag, position, delta):
    '''!@brief              Routinely checks for user input and acts accordingly.
        @details            This generator function, given the right set of
                            inputs, is able to constantly check for user input
                            and will change state based on what the user specifies.
                            It reads shared variables that taskEncoder writes to
                            in order to relay data to the user. There are 7 various
                            states that taskUser can operate in. State 0 initializes
                            the program and prints a list of commands the user can
                            input. State 1 is the command center that waits for user
                            input in order to switch the state. State 2 calls
                            for the encoder to be zeroed. State 3 calls for the
                            position value and prints it. State 4 calls for the
                            delta value and prints it. State 5 takes position
                            data for 30 seconds and State 6 will print it. State 5
                            can be interrupted by pressing 's' and the function
                            will start State 6 early.
        @param taskName     The arbitrary, chosen name of the task.
        @param period       The time interval at which this task should operate.
        @param zFlag        A shared variable that means the encoder should be zeroed.
        @param position     A shared variable that holds the current position value.
        @param delta        A shared variable that holds the last delta value.
    '''
    
    ## @brief Represents the initialization state of the task.
    #
    S0_INIT = micropython.const(0)
    
    ## @brief Represents the command center state of the task.
    #
    S1_CMD = micropython.const(1)
    
    ## @brief Represents the zeroing state of the task.
    #
    S2_ZERO = micropython.const(2)
    
    ## @brief Represents the position calling state of the task.
    #
    S3_POS = micropython.const(3)
    
    ## @brief Represents the delta calling state of the task.
    #
    S4_DEL = micropython.const(4)
    
    ## @brief Represents the data collecting state of the task.
    #
    S5_DATA = micropython.const(5)
    
    ## @brief Represents the data printing state of the task.
    #
    S6_PRINT = micropython.const(6)
    
    ## @brief       Denotes the state that the task is currently running in.
    #  @details     taskUser has 6 possible states. State 0 is just initializing
    #               the program and setting up the user interface. State 1
    #               is the command center and waits for user input. Pressing
    #               'z' will transition to State 2 which zeros the encoder.
    #               Pressing 'p' will transtion to State 3 which prints the
    #               position of the encoder. Pressing 'd will transition to
    #               State 4 which prints the delta of the encoder. All of these
    #               states essentially transfer back to State 1 automatically.
    #               State 5 is triggered by pressing G and will record encoder
    #               position data for 30 seconds or until s is pressed. State
    #               5 always transitions to State 6 which will output all the
    #               recorded data. State 6 cannot be interrupted and ends when
    #               all the data is output.
    #
    state = S0_INIT
    
    ## @brief Contains serial inputs from the keyboard.
    #
    serport = pyb.USB_VCP()
    
    ## @brief The time at which the task is first run.
    #
    start_time = ticks_us()
    
    ## @brief The time at which the task should start its next cycle.
    #
    next_time = ticks_add(start_time, period)
    
    ## @brief An array of time values taken during data collection.
    #
    timeArray = array.array('f', 3001*[0])
    
    ## @brief An array of position values taken during data collection.
    #
    posArray = array.array('f', 3001*[0])
    velArray = array.array('f', 3001*[0])
    
    while True:
        ## @brief The time at the beginning of the current task run.
        #
        current_time = ticks_us()
        
        # Check if alarm clock went off
        if ticks_diff(current_time, next_time) >= 0:
            next_time = ticks_add(next_time, period)
            
            # Initialize
            if state == S0_INIT:
                print('Initializing...')
                _printHelp()
                state = S1_CMD
                
            # Command Center
            elif state == S1_CMD:
                if serport.any():
                    ## @brief Character data from user's keyboard inputs.
                    #
                    charIn = serport.read(1).decode()
                    
                    #Set state according to user input
                    if charIn in {'z','Z'}:
                        print('Zeroing the Encoder')
                        state = S2_ZERO
                        yield zFlag.write(True)

                    elif charIn in {'p','P'}:
                        state = S3_POS
                        yield None
                        
                    elif charIn in {'d','D'}:
                        state = S4_DEL
                        yield None
                        
                    elif charIn in {'g','G'}:
                        state = S5_DATA
                        ## @brief Index used for data collection purposes.
                        #
                        idx = 0
                        yield None
                        
                    elif charIn in {'s','S'}:
                        yield print('S is only valid during data collection!')
                        
                    else:
                        print(f'{charIn} is not a valid input. Please try again.')
                        yield None
                else:
                    yield None
                    
            # Zeroing state
            elif state == S2_ZERO:
                if not zFlag.read():
                    print('Encoder value set to zero.')
                    state = S1_CMD
                    yield None
                    
                else:
                    yield None
            
            # Position grabbing state
            elif state == S3_POS:
                print(f'The encoder is at {position.read()}')
                state = S1_CMD
                yield None
            
            # Delta grabbing state
            elif state == S4_DEL:
                print(f'The last delta was {delta.read()}')
                state = S1_CMD
                yield None

            # Data collection state
            elif state == S5_DATA:
                
                # start a data timer
                if idx == 0:
                    print('Press s/S to stop data collection early.')
                    posArray[idx] = position.read()
                    velArray[idx] = delta.read()
                    timeArray[idx] = 0
                    t = ticks_us()
                    idx += 1
                    yield None
                    
                # check for interrupt
                elif serport.any():
                    charIn = serport.read(1).decode()
                    
                    if charIn in {'s','S'}:
                        posArray[idx] = position.read()
                        velArray[idx] = delta.read()
                        timeArray[idx] = int(ticks_diff(ticks_us(),t)/1_000_000)
                        state = S6_PRINT
                        ## @brief Keeps track of how many data points have been printed.
                        #
                        numPrinted = 0
                        yield None
                        
                    else:
                        posArray[idx] = position.read()
                        velArray[idx] = delta.read()
                        timeArray[idx] = int(ticks_diff(ticks_us(),t)/1_000_000)
                        idx += 1
                        yield None
                        
                # check for end case    
                elif idx == 3000:
                    posArray[idx] = position.read()
                    velArray[idx] = delta.read()
                    timeArray[idx] = 30
                    numPrinted = 0
                    state = S6_PRINT
                    yield None
                
                else:
                    posArray[idx] = position.read()
                    velArray[idx] = delta.read()
                    timeArray[idx] = ticks_diff(ticks_us(),t)/1_000_000
                    idx += 1
                    yield None
            
            # Data printing state
            elif state == S6_PRINT:
                if numPrinted == idx:
                    print(f'{timeArray[numPrinted]:.2f}, {posArray[numPrinted]:.2f}, {velArray[numPrinted]:.2f}')
                    state = S1_CMD
                    print('End of Data Collection')
                    _printHelp()
                    yield None
                   
                else:
                    print(f'{timeArray[numPrinted]:.2f}, {posArray[numPrinted]:.2f}, {velArray[numPrinted]:.2f}')
                    numPrinted += 1
                    yield None
             
        else:
            yield None
        
def _printHelp():
    print('+---------------------------------------------------------+')
    print('|    User Interface:  Please select an input              |')
    print('+---------------------------------------------------------+')
    print('+---------------------------------------------------------+')
    print('| Input |                     Function                    |')
    print('+---------------------------------------------------------+')
    print('| z / Z |  Zero the encoder.                              |')
    print('+---------------------------------------------------------+')
    print('| p / P |  Retrieve encoder position.                     |')
    print('+---------------------------------------------------------+')
    print('| d / D |  Retrieve encoder delta.                        |')
    print('+---------------------------------------------------------+')
    print('| g / G |  Retrieve encoder position for thirty seconds.  |')
    print('+---------------------------------------------------------+')
    print('| s / S |  Stop data collection prematurely.              |')
    print('+---------------------------------------------------------+')