
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <si5351.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static Si5351 si5351;

/**
 * Used for creating moving averages
 */
template <int sz> class Window {
public:

  Window() {
    for (int i = 0; i < _size; i++)
      _buffer[i] = 0;
  }

  int getMovingAverage() const {
    int result = 0;
    for (int i = 0; i < _size; i++)
      result += _buffer[i];
    return result / _size;
  }

  void add(int sample) {
    _buffer[_ptr++] = sample;
    // Do wrap-around
    if (_ptr == _size) 
      _ptr = 0;
  }

private:

  int _size = sz;
  int _buffer[sz];
  int _ptr = 0;
};

/**
 * An abstract interface used for reporting motion of a rotary encoder
 */
class EncoderEvent {
public:
  
  virtual void moved(int increment) = 0;
};

/**
 * Used for managing rotatary encoders
 */
template <int WINDOW_SIZE> class Encoder {
public:

  Encoder(unsigned int intervalUs, int pin0, int pin1, EncoderEvent* event)
  : _intervalUs(intervalUs),
    _pin0(pin0),
    _pin1(pin1),
    _event(event) {    
  }

  void poll() {

    unsigned long nowUs = micros();

    // We sample every 50us and smooth over a window
    if (nowUs - _timer0 > _intervalUs) {
  
      // Reset timer
      _timer0 = nowUs;
      
      // Add a sample to the low-pass filter
      _lag0.add((digitalRead(_pin0) == HIGH) ? 255 : 0);
      _lag1.add((digitalRead(_pin1) == HIGH) ? 255 : 0);
  
      // Make up a two-bit reading
      unsigned int sample = 0;
  
      if (_lag0.getMovingAverage() >= 128) {
        sample |= 2;    
      }
      if (_lag1.getMovingAverage() >= 128) {
        sample |= 1;
      }

      // If the encoder's state has changed then something is moving
      if (sample != _lastSample) {

         // Shift up and apply the latest sample on the right
        _accumulator = ((_accumulator << 2) & 0b11111100) | sample;
  
        // Error check the Grey code and determine movement
        if ((_accumulator & 0b11111111) == 0b11100001) {
          _event->moved(1);
        } else if ((_accumulator & 0b11111111) == 0b11010010) {
          _event->moved(-1);
        }
        
        _lastSample = sample;        
      }
    }    
  }

private: 

  const unsigned int _intervalUs;
  const int _pin0, _pin1;
  EncoderEvent* const _event;
  Window<WINDOW_SIZE> _lag0;
  Window<WINDOW_SIZE> _lag1;
  
  unsigned long _timer0 = 0;
  unsigned int _lastSample = 0;
  unsigned int _accumulator = 0;
};

class TuningEvent {
public:

  virtual void setFreq(unsigned long hz) = 0;
};

class EncoderEventImpl : public EncoderEvent {
public:

  EncoderEventImpl(TuningEvent* ev)
  : _ev(ev) {
  }

  void moved(int inc) {
    if (inc == 1) {
      //Serial.println(" CW");
      _freq += 500;
      updateDisplay();
      _ev->setFreq(_freq);
    } else if (inc == -1) {
      //Serial.println("CCW");
      _freq -= 500;
      updateDisplay();
      _ev->setFreq(_freq);
    }
  }

private:

  void updateDisplay() {

    // Clear the buffer
    display.clearDisplay();
  
    display.setTextSize(2);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text

    int m = _freq / 1000000;
    int k = (_freq % 1000000) / 1000;
    int k5 = (_freq % 1000) / 100;

    char buf[32];
    sprintf(buf,"%2d", m);
    display.setCursor(0,16);
    display.println(buf);

    sprintf(buf,"%03d.%1d", k, k5);
    display.setCursor(28,16);
    display.println(buf);
  
    display.display();    
  }

  unsigned int _freq = 7100000;
  TuningEvent* _ev;
};

class TuningEventImpl : public TuningEvent {
public:

  void setFreq(unsigned long hz) {
    Serial.println(hz);
    unsigned long long f100 = (unsigned long long)hz * 100ULL;
    si5351.set_freq(f100, SI5351_CLK1);
  }
};

static TuningEventImpl tuningImpl;
static EncoderEventImpl encoderEventImpl(&tuningImpl);
static Encoder<16> encoder(500, 11, 12, &encoderEventImpl);

void setup() {

  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  // NOTE: This was adjusted on the SI5351 module with the bad CLK0
  unsigned int correction = 11000;
  
  int i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, correction);
  if (!i2c_found) {
    Serial.println(F("Si5351 allocation failed"));
  }

  Serial.println("Initialized");
}

void loop() {
  encoder.poll();
}
