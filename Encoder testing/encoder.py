'''!@file           encoder.py
    @brief          Creates an encoder class to set up an encoder and give it 
                    the ability to carry out several methods.
    @details        Given the correct pin objects and timer number, an encoder
                    object will be created. With this encoder object, there are
                    four main methods that can be performed. The actions that
                    can be performed are updating the position, returning the 
                    position, returning the last change in position, and setting
                    the position back to zero. When run as the main file, will
                    update and output the encoder's position every second.
                    
                    Note: our method of updating requires fixing the position 
                    value for over/underflow. To do so we have to distinguish
                    a tick value at which the encoder deems the change to be
                    impossible through normal rotation conditions. This
                    inherently can be broken if the motor moves too fast or
                    updates too slowly. In order to choose a proper update
                    frequency for your motor, please follow the calculations below.

                    @image html Lab2FreqCalc.JPG
                    
                    For a class diagram, reference the following:
                        
                    @image html Lab2ClassDiagram.JPG
    @author         Sean Wahl
    @author         Grant Gabrielson
    @date           February 1, 2022
'''
import pyb, time, math

class Encoder:
    '''!@brief               Creates an encoder object that can perform basic functions.
        @details             An encoder object can be created by calling for
                             the encoder class and referencing pin objects
                             and a timer number. These pins and timer should
                             correspond to the correct encoder ports on the 
                             Nucleo. The methods that the encoder can enact are
                             updating, zeroing, and returning position and change
                             in position.
        @author              Sean Wahl
        @author              Grant Gabrielson
        @date                February 1, 2022
    '''

    def __init__(self, ch1_pin, ch2_pin, timNum, CPR):
        '''!@brief              Initializes a new encoder.
            @details            Takes in pin objects and a timer number in order
                                to link them to an encoder. Once referenced,
                                the encoder will be set up to read tick values
                                from its timer that represent its rotation.
                                This rotation value resets at 65535.
            @param ch1_pin      Pin object corresponding to the first pin of 
                                the encoder on the Nucleo.
            @param ch2_pin      Pin object corresponding to the second pin of 
                                the encoder on the Nucleo.
            @param timNum       Timer number compatible with the encoder.
        '''
        ## Pin object 1 now connected to the encoder.
        #
        self._pin1 = pyb.Pin(ch1_pin, pyb.Pin.IN)
        
        ## Pin object 2 now connected to the encoder.
        #
        self._pin2 = pyb.Pin(ch2_pin, pyb.Pin.IN)
        
        ## Links the timer number to the encoder and sets the correct timing values.
        #
        self._tim = pyb.Timer(timNum, period=65535, prescaler=0)
        
        ## Links the timer channel 1 to the correct encoder pin.
        #
        self._ch1 = self._tim.channel(1, pyb.Timer.ENC_AB, pin = self._pin1)
        
        ## Links the timer channel 2 to the correct encoder pin.
        #
        self._ch2 = self._tim.channel(2, pyb.Timer.ENC_AB, pin = self._pin2)
        
        ## Current tick value of the encoder.
        #
        self._cur = 0
        
        ## Corrected position value of the encoder.
        #
        self.position = 0

        self._tlast = time.ticks_us()

        self._CPR = CPR
        self._timeList = [0]*10
        self._deltaList = [0]*10

        
    def update(self):
        '''!@brief      Updates the position value of the encoder.
            @details    Uses the previous and current tick position of the
                        encoder in order to find the new position value.
                        Can adjust for over/underflow if necessary, and stores
                        the last delta and the new position value. The encoder
                        should perform this function often. See the encoder.py
                        details for more.
        '''
        ## Sets the previous tick value to be the last current value.
        #
        self._prev = self._cur
        
        ## Retrieves a new current tick value.
        #
        self._cur = self._tim.counter()
        
        # Check if improbable rotation condition has been met.
        if abs(self._cur - self._prev) < 32768:
            ## The last change in position of the encoder.
            self.delta = self._cur - self._prev
            
        elif self._cur < self._prev:
            self.delta = self._cur + 65535 - self._prev
            
        else:
            self.delta = self._cur - 65535 - self._prev
        
        # Adjust the position value as needed.
        self.position += self.delta
        
        self._tdiff = time.ticks_diff(time.ticks_us(), self._tlast)/1_000_000
        self._tlast = time.ticks_us()
        self._timeList.append(self._tdiff)
        self._timeList.pop(0)
        self._deltaList.append(self.delta)
        self._deltaList.pop(0)
        

        
    def get_position(self):
        '''!@brief      Retrieves the position value of the encoder.
            @details    By calling the get_position() method, the encoder will
                        send out the last recorded position value from the
                        update method in ticks.
            @return     Position value of the encoder.
        '''
        return self.position
        #   *2*math.pi/(4*self._CPR)
    
    def zero(self):
        '''!@brief      Sets the encoder position to zero.
            @details    Overwrites the current position value and changes it
                        to be zero. This is non-reversible and the old position
                        is not retrievable unless the data was being collected.
        '''
        self.position = 0
        
    def get_delta(self):
        '''!@brief      Retrieves the last change in position of the encoder.
            @details    By calling the get_delta() method, the encoder will
                        send out the last change in position value in ticks.
            @return     Last change in position of the encoder.
        '''
        return (sum(self._deltaList)/sum(self._timeList))*2*math.pi/(4*self._CPR)
        #self.delta*2*math.pi/(4*self._CPR*self._tdiff)

if __name__ == '__main__':
    # Sets up an encoder hooked up to pins B6 and B7 and calls for it to update
    # and return its position every second.
    _encoder_1 = Encoder(pyb.Pin.cpu.B6, pyb.Pin.cpu.B7, 4)
    
    while True:
        try:
            _encoder_1.update()
            _encoder_1.get_position()
            time.sleep(1)
        except KeyboardInterrupt:
            break
            
    print('stopping')
        
    
