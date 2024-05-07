// DEFINITIONS
#define G               9.8
#define FLEX_1          A4
#define FLEX_2          A3
#define FLEX_3          A2
#define FLEX_4          33
#define FLEX_5          37                                                            


// FLEX SENSOR GLOBAL VARIABLES
int FLEX_PINS[] = {FLEX_1, FLEX_2, FLEX_3, FLEX_4, FLEX_5};
int calibrated_values[] = {0, 0, 0, 0, 0};


// OUTPUT BUFFER AND VARIABLE
String data;
char buffer[2056];


// Farward declaration
void getFlexOffsets_median();

//-------------------------- MAIN ROUTINE --------------------------//
void setup()
{
    // INITIALIZE SERIAL 
  Serial.begin(115200); // BAUD_RATE = 115200
  while (!Serial);    // wait until serial is available

  pinMode(LED_BUILTIN, OUTPUT);

  // Indicate on initialization 
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  // INITIATE FLEX SENSOR OFFSET CALCULATION << THESE VALUES SHOULD BE SET AFTER CALIBRATION FOR EACH SENSOR MODULES >>
  getFlexOffsets_median();
  for (int i = 0; i < 5; i++)
  {
    Serial.println(calibrated_values[i]);
  }
    Serial.println("Calibrated flex sensors");
}

void loop(){}

// FLEX OFFSET CALCULATION ROUTINE -- MEDIAN
void getFlexOffsets_median()
{
  Serial.println("HERE");
  int flex[5][100];
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 100; j++)
    {
      flex[i][j] = 0;
    }
  }

  for (int i = 0; i < 100; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      flex[j][i] = analogRead(FLEX_PINS[j]);
      for (int k = 0; k < i; k++)
      {
        for (int l = 1; l < 100; l++)
        {
          if (flex[j][l] > flex[j][l - 1])
          {
            int temp = flex[j][l];
            flex[j][l] = flex[j][l - 1];
            flex[j][l - 1] = temp;
          }
        }
      }
    }
    delay(70);
  }

  for (int i = 0; i < 5; i++)
  {
    calibrated_values[i] = flex[i][50];
  }
}
