/* USER CODE BEGIN Header */
/* ... keep Cube header ... */
/* USER CODE END Header */
#include "main.h"
#include <string.h>
#include <stdio.h>

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* ---------- HC-SR04 capture state ---------- */
volatile uint32_t ic_rising = 0;
volatile uint32_t ic_falling = 0;
volatile uint32_t echo_us = 0;
volatile uint8_t  ic_state = 0;
volatile uint8_t  echo_done = 0;

/* ---------- DHT11 latest values ---------- */
static float g_tempC = 25.0f;
static float g_hum   = 0.0f;

/* ---------- Filtering ---------- */
static const float MIN_CM = 5.0f;
static const float MAX_CM = 200.0f;

static const float CAL_OFFSET_CM   = -0.25f;
static const float EMA_ALPHA       = 0.25f;
static const float JUMP_REJECT_CM  = 20.0f;

static uint8_t hasSmooth = 0;
static float smoothCm = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void delay_us(uint32_t us);
static uint32_t micros(void);

/* LCD (PCF8574) */
#define LCD_ADDR      (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define LCD_EN        0x04
#define LCD_RS        0x01
static void LCD_WriteExpander(uint8_t data);
static void LCD_PulseEnable(uint8_t data);
static void LCD_Write4Bits(uint8_t nibble, uint8_t rs);
static void LCD_Send(uint8_t value, uint8_t rs);
static void LCD_Cmd(uint8_t cmd);
static void LCD_Data(uint8_t data);
static void LCD_Init(void);
static void LCD_Clear(void);
static void LCD_SetCursor(uint8_t col, uint8_t row);
static void LCD_Print(const char *s);

/* DHT11 */
static void DHT_PinOutput(void);
static void DHT_PinInput(void);
static int  DHT11_Read(float *tempC, float *hum);

/* HC-SR04 */
static void HCSR04_Trigger(void);
static int32_t HCSR04_ReadEchoUs(uint32_t timeout_ms);

/* Utilities */
static float speed_of_sound_ms(float tempC);
static float median5(float a[5]);
static float HCSR04_ReadDistanceMedian(float tempC);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* ---------- DWT microsecond timing ---------- */
static void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static uint32_t micros(void) {
  return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}
static void delay_us(uint32_t us) {
  uint32_t start = micros();
  while ((micros() - start) < us) { }
}

/* ---------- LED 2 blinks/sec (non-blocking) ---------- */
static void LED_Blink2PerSecond_Task(void) {
  static uint32_t last = 0;
  static uint8_t step = 0;
  uint32_t now = HAL_GetTick();

  const uint16_t dur[4] = {80, 170, 80, 670}; //

  if ((now - last) >= dur[step]) {
    last = now;
    // PC13 often inverted on Blue Pill boards (LOW=ON)
    if (step == 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    if (step == 1) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    if (step == 2) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    if (step == 3) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    step = (step + 1) % 4;
  }
}

/* ---------- TIM3 Input Capture callback ---------- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    if (ic_state == 0) {
      ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
      ic_state = 1;
    } else {
      ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

      uint32_t diff;
      if (ic_falling >= ic_rising) diff = ic_falling - ic_rising;
      else diff = (0x10000U - ic_rising) + ic_falling;

      echo_us = diff;
      echo_done = 1;

      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
      ic_state = 0;
    }
  }
}

/* ---------- HC-SR04 ---------- */
static void HCSR04_Trigger(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

static int32_t HCSR04_ReadEchoUs(uint32_t timeout_ms) {
  echo_done = 0;
  ic_state = 0;
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  HCSR04_Trigger();

  uint32_t t0 = HAL_GetTick();
  while (!echo_done) {
    if ((HAL_GetTick() - t0) > timeout_ms) return -1;
  }
  return (int32_t)echo_us;
}

static float speed_of_sound_ms(float tempC) {
  return 331.3f + 0.606f * tempC;
}

static float median5(float a[5]) {
  // simple sort for 5 items
  for (int i=0;i<5;i++) {
    for (int j=i+1;j<5;j++) {
      if (a[j] < a[i]) { float t=a[i]; a[i]=a[j]; a[j]=t; }
    }
  }
  return a[2];
}

static float HCSR04_ReadDistanceMedian(float tempC) {
  float v[5];
  float c = speed_of_sound_ms(tempC);

  for (int i=0;i<5;i++) {
    int32_t us = HCSR04_ReadEchoUs(60);
    if (us < 0) v[i] = -1.0f;
    else {
      float cm = ((float)us * c) / 20000.0f;
      if (cm < 1.0f || cm > 500.0f) cm = -1.0f;
      v[i] = cm;
    }
    HAL_Delay(40);
  }

  int valid = 0;
  for (int i=0;i<5;i++) if (v[i] > 0) valid++;
  if (valid < 3) return -1.0f;

  for (int i=0;i<5;i++) if (v[i] <= 0) v[i] = 9999.0f;
  float med = median5(v);
  if (med >= 9999.0f) return -1.0f;
  return med;
}

/* ---------- LCD via PCF8574 (common mapping) ---------- */
static void LCD_WriteExpander(uint8_t data) {
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, 50);
}
static void LCD_PulseEnable(uint8_t data) {
  LCD_WriteExpander(data | LCD_EN);
  delay_us(1);
  LCD_WriteExpander(data & ~LCD_EN);
  delay_us(50);
}
static void LCD_Write4Bits(uint8_t nibble, uint8_t rs) {
  uint8_t data = LCD_BACKLIGHT | (rs ? LCD_RS : 0x00) | (nibble << 4);
  LCD_WriteExpander(data);
  LCD_PulseEnable(data);
}
static void LCD_Send(uint8_t value, uint8_t rs) {
  LCD_Write4Bits(value >> 4, rs);
  LCD_Write4Bits(value & 0x0F, rs);
}
static void LCD_Cmd(uint8_t cmd) { LCD_Send(cmd, 0); }
static void LCD_Data(uint8_t data) { LCD_Send(data, 1); }

static void LCD_Init(void) {
  HAL_Delay(50);
  LCD_Write4Bits(0x03, 0); HAL_Delay(5);
  LCD_Write4Bits(0x03, 0); HAL_Delay(5);
  LCD_Write4Bits(0x03, 0); HAL_Delay(1);
  LCD_Write4Bits(0x02, 0);

  LCD_Cmd(0x28);
  LCD_Cmd(0x0C);
  LCD_Cmd(0x06);
  LCD_Clear();
}
static void LCD_Clear(void) { LCD_Cmd(0x01); HAL_Delay(2); }

static void LCD_SetCursor(uint8_t col, uint8_t row) {
  static const uint8_t row_offsets[] = {0x00, 0x40};
  LCD_Cmd(0x80 | (col + row_offsets[row]));
}
static void LCD_Print(const char *s) {
  while (*s) LCD_Data((uint8_t)*s++);
}

/* ---------- DHT11 (PA1) bit-bang ---------- */
static void DHT_PinOutput(void) {
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_1;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);
}
static void DHT_PinInput(void) {
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_1;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);
}


static int wait_pin(GPIO_PinState st, uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) != st) {
    if ((micros() - t0) > timeout_us) return 0;
  }
  return 1;
}

static int DHT11_Read(float *tempC, float *hum) {
  uint8_t data[5] = {0};


  DHT_PinOutput();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(18);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  delay_us(30);
  DHT_PinInput();


  if (!wait_pin(GPIO_PIN_RESET, 200)) return 0;
  if (!wait_pin(GPIO_PIN_SET,   200)) return 0;
  if (!wait_pin(GPIO_PIN_RESET, 200)) return 0;


  for (int i=0;i<40;i++) {

    if (!wait_pin(GPIO_PIN_SET, 100)) return 0;
    uint32_t t0 = micros();
    if (!wait_pin(GPIO_PIN_RESET, 120)) return 0;
    uint32_t high_us = micros() - t0;

    uint8_t bit = (high_us > 50) ? 1 : 0;
    data[i/8] <<= 1;
    data[i/8] |= bit;
  }

  uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (sum != data[4]) return 0;


  *hum   = (float)data[0];
  *tempC = (float)data[2];
  return 1;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  DWT_Init();

  // Start input capture interrupt for ECHO
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

  // LCD init + banner
  LCD_Init();
  LCD_SetCursor(0,0); LCD_Print("HC-SR04 + DHT11");
  LCD_SetCursor(0,1); LCD_Print("Range 5..200cm ");
  HAL_Delay(1200);
  LCD_Clear();


  uint32_t lastDht = 0;
  uint32_t lastLcd = 0;
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    LED_Blink2PerSecond_Task();


    if ((HAL_GetTick() - lastDht) >= 2000) {
      lastDht = HAL_GetTick();
      float t,h;
      if (DHT11_Read(&t, &h)) {
        g_tempC = t;
        g_hum   = h;
      }
    }


    if ((HAL_GetTick() - lastLcd) >= 250) {
      lastLcd = HAL_GetTick();

      float dcm = HCSR04_ReadDistanceMedian(g_tempC);

      if (dcm > 0) dcm += CAL_OFFSET_CM;


      LCD_SetCursor(0,0);
      LCD_Print("D:              ");
      LCD_SetCursor(2,0);

      if (dcm > 0 && dcm < MIN_CM) {
        LCD_Print("Too close");
        hasSmooth = 0;
      }
      else if (dcm > MAX_CM) {
        LCD_Print("Out of range");
        hasSmooth = 0;
      }
      else if (dcm < 0) {

        if (hasSmooth) {
          int di = (int)(smoothCm + 0.5f);
          char b[17];
          snprintf(b, sizeof(b), "%d cm        ", di);
          LCD_Print(b);
        } else {
          LCD_Print("Out of range");
        }
      }
      else {

        if (!hasSmooth) {
          smoothCm = dcm;
          hasSmooth = 1;
        } else {
          float diff = dcm - smoothCm;
          if (diff < 0) diff = -diff;
          if (diff <= JUMP_REJECT_CM) {
            smoothCm = (1.0f - EMA_ALPHA) * smoothCm + EMA_ALPHA * dcm;
          }
        }
        int di = (int)(smoothCm + 0.5f);
        char b[17];
        snprintf(b, sizeof(b), "%d cm        ", di);
        LCD_Print(b);
      }


      LCD_SetCursor(0,1);
      char th[17];
      snprintf(th, sizeof(th), "T:%2dC H:%2d%%   ",
               (int)(g_tempC + 0.5f), (int)(g_hum + 0.5f));
      LCD_Print(th);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
