'''!@file           Lab2Main.py
    @brief          Calls functions in order to create a working encoder data collector.
    @details        Periodically calls taskUser and taskEncoder to work together
                    using shared variables and generator functions. Creates 
                    zFlag, position, and delta which are sent between taskUser 
                    and taskEncoder to relay information. The main purpose of 
                    this main file is to continuously try to run each task over
                    and over again. The internal task periods will take care
                    of when they should actually run. To stop the program press
                    'Ctrl+C' and to reset the Nucleo press 'Ctrl+D'. For a task
                    diagram, reference the following image:
                        
                    @image html Lab2TaskDiagram.JPG
    @author         Sean Wahl
    @author         Grant Gabrielson
    @date           February 2, 2022
'''
import shares, taskUser, taskEncoder

## @brief Shared variable to represent when the encoder should be zeroed.
#
zFlag = shares.Share(False)

## @brief Shared variable to store the current position of the encoder.
#
position = shares.Share(0)

## @brief Shared variable to store the last delta of the encoder.
delta = shares.Share(0)

if __name__ == '__main__':
    ## @brief List containing taskUser and taskEncoder functions
    #
    tasklist = [taskUser.taskUserFunction('Task User', 10_000, zFlag, position, delta),
                taskEncoder.taskEncoderFunction('Task Encoder', 10_000, zFlag, position, delta)]
    
    while True:
        try:
            for task in tasklist:
                next(task)
        except KeyboardInterrupt:
            break