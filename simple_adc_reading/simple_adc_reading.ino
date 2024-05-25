/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadAnalogVoltage
*/

const float VREF = 3.3F;
const unsigned int ADC_RES_BIT = 12;
const float ADC_NUM_SAMPLES = 2 << (ADC_RES_BIT - 1);
const float VOLTAGE_DIVIDER_FACTOR = 0.24F;

// the setup routine runs once when you press reset:
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // change the ADC resolution
  analogReadResolution(ADC_RES_BIT);
  analogReference(AR_DEFAULT);
}

// the loop routine runs over and over again forever:
void loop()
{
  // read the input on analog pin 0 and convert to voltage range (0 - VREF):
  const int analog_val = analogRead(A0);
  const float voltage = analog_val * (VREF / ADC_NUM_SAMPLES);
  // print out the value you read:
  Serial.print("Measured value: ");
  Serial.print(analog_val);
  Serial.print(", voltage: ");
  Serial.println(voltage);
  Serial.print("Scaled back voltage: ");
  Serial.println(voltage / VOLTAGE_DIVIDER_FACTOR);
  delay(100);
}