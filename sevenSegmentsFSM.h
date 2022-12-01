
#include <stdint.h>

// what character is being displayed
enum Phases
{
    SevenSegDisplayType,      // display either "P" or "F"
    SevenSegDisplayValue,     // display percentage or error code (the name of the constant is confusing but we use this state to display the whole number, another counter is used to track exactly the index of the current number to be displayed, otherwise we would have a lot of states, or may be it would make more sense to rename it?)
    SevenSegDisplayFinlaBlank // display a blank character for 500ms (not to confuse with the 100ms flickering blank)
};

uint8_t SevenSegNumbers[] = {
    // seven segments coded numbers
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111  // 9
};

const uint8_t SevenSegP = 0b1110011; // letter "P" coded in seven segments
const uint8_t SevenSegF = 0b1110001; // letter "F" coded in...

// extract a digit form a number at a given index, then return its seven segment reperesentation
uint8_t extractSevenSegDigit(uint32_t value, uint8_t index)
{
    for (int i = 0; i < index; i++)
        value /= 10;
    value %= 10;
    return SevenSegNumbers[value]; // convert number into 7 seg representation
}

class SevenSegmentsFSM
{
private:
    Phases _phase;
    uint8_t _charIndex; // the index of a digit from the error code being displayed during SevenSegDisplayValue phase
    uint8_t _numberOfChars;
    uint8_t _flicker;
    uint8_t _endFlicker;
    uint8_t _result;
    uint8_t _value;

public:

    void (*OnLastPhaseDone)();
    void (*OnFirstPhaseStarted)();

    uint8_t isFlickering() { return _flicker; }

    uint8_t isFlickerDone(uint8_t clear)
    {
        uint8_t endFlicker = _endFlicker;
        if (clear)
            _endFlicker = 0;
        return endFlicker;
    }

    void SetValue(uint8_t value)
    {
        _value = value;
    }

    SevenSegmentsFSM(uint8_t numberOfChars)
    {
        _numberOfChars = numberOfChars - 1;
        Reinit();
    }

    uint8_t LastResult() { return _result; }

    Phases GetCurrentPhase() { return _phase; }

    // returns 1 when the FSM ends, 0 otherwise
    uint8_t Execute()
    {
        switch (_phase)
        {
        case SevenSegDisplayType:          // just started displaying the error code
            if (OnFirstPhaseStarted) // execute the event handler if existant
                OnFirstPhaseStarted();
            _charIndex = _numberOfChars;   // reinit the index
            _phase = SevenSegDisplayValue; // the next time this function is called it shall process SevenSegDisplayValue
            _flicker = 1;                  // when SevenSegDisplayValue is being processed a short blank will be dispalyed
            _result = SevenSegF;           // the letter 'F' is to be displayed
            break;

        case SevenSegDisplayValue: // dispaly the error code
        {
            if (isFlickering()) // a flicker previously requested
            {
                _result = 0;     // return blank character
                _flicker = 0;    // no more flicker
                _endFlicker = 1; // tells "calculate7Seg" to display this charcter for only 100ms
            }
            else
            {
                _flicker = 1; // a character is being displayed now, so the next time flicker
                // uint8_t value = errors[0];                                    // get the first error
                _result = extractSevenSegDigit(_value, _charIndex); // dispayed the digit with index of "_charIndex"
                if (_charIndex == 0)                                // is it the last digit to display? then shift the errors stack
                {
                    _phase = SevenSegDisplayFinlaBlank; // move to the next phase of display
                    _flicker = 0;                       // no flicker, the next phase is just blank character for 500ms
                }
                else
                {
                    _charIndex--; // not the last digit? then move to the next one
                }
            }
        }
        break;

        case SevenSegDisplayFinlaBlank: // display a blank character for 500ms
            // StateOfSevenSeg = SevenSegDisplayingWaterLevel; // the last digit of this error is displayed, switching back to water level display, if another error is in stack "calculate7Seg" will call this again
            // reset the state of PhaseOfSevenSeg
            // PhaseOfSevenSeg = SevenSegDisplayType;
            // SevenSegFlicker = 0;
            Reinit(); // finalize, the result is a blank character
            if (OnLastPhaseDone) // execute the even handler is existing
                OnLastPhaseDone();
            return 1; // tell that the machine finished executing all its states
            break;
        }
        return 0; // still other states to be executed next
    }

    void Reinit()
    {
        _charIndex = _numberOfChars;
        _flicker = _endFlicker = 0;
        _phase = SevenSegDisplayType;
        _result = 0;
    }
};