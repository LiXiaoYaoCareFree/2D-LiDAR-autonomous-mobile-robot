/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ???? + ADC + RPLIDAR + MPU6500
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu6500.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "rader.h"
#define MAX_SPEED 500
#define SPEED_STEP 100
#define SPEED_UPDATE_INTERVAL 50
#define TURN_DURATION 800
#define TURN_SPEED 300
#define DIR_STOP 0
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_UTURN 5
#define PPR 360
#define SAMPLE_TIME_MS 100
#define DMA_BUFFER_SIZE 256
#define DATA_PACKET_SIZE 5
#define ANGLE_FILTER_THRESHOLD 1.0f
#define MIN_VALID_DISTANCE 50.0f
#define MAX_VALID_DISTANCE 12000.0f
#define LIDAR_TIMEOUT_THRESHOLD 500
// ==== PID????????? BEGIN ====
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float last_error;
    float output;
} PID_Controller;

float PID_Compute(PID_Controller* pid, float measured) {
    float error = pid->setpoint - measured;
    pid->integral += error;
    float derivative = error - pid->last_error;
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->last_error = error;

    // ????
    if (pid->output > MAX_SPEED) pid->output = MAX_SPEED;
    if (pid->output < 0) pid->output = 0;

    return pid->output;
}

// ?????????PID???
PID_Controller pidA = {0.8f, 0.01f, 0.1f, 0, 0, 0, 0};
PID_Controller pidB = {0.85f, 0.01f, 0.1f, 0, 0, 0, 0};

// ????,??A?B???????
float rpmA = 0.0f;
float rpmB = 0.0f;
// ==== PID????????? END ====

uint8_t bluetooth_rx_data = 0;
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;

uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

uint8_t is_turning_angle = 0;
float turning_angle_accum = 0.0f;
uint8_t turning_direction = DIR_STOP;  // DIR_LEFT ? DIR_RIGHT
uint8_t is_turning_u = 0;               // ?????
float turning_angle_u_accum = 0.0f;     // ??????


int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;
char uart_buf[100];

uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
uint8_t lidar_dataBuffer[DATA_PACKET_SIZE];
uint8_t lidar_dataIndex = 0;
uint32_t lidar_rxIndex = 0;
volatile uint8_t lidar_process_packet = 0;
float lastLidarAngle = 0.0f;
uint32_t lastLidarRxTime = 0;
///////////////////////////////////////////////////////

static uint8_t uart4_rx_con=0;       //½ÓÊÕ¼ÆÊýÆ÷
static uint8_t uart4_rx_chksum;      //Òì»òÐ£Ñé
static uint8_t uart4_rx_buf[100];     //½ÓÊÕ»º³å
static uint8_t uart4_tx_buf[10];     //½ÓÊÕ»º³å


uint8_t uart4_rx_data = 0;  // ???????

//É¨ÃèÒ»È¦µÄÀ×´ïÊý¾Ý

LaserPointTypeDef ax_ls_point[250];

///////////////////////////////////////////////////////


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart4;
void SystemClock_Config(void);
void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);
void SendConnectionNotification(void);
void ProcessLidarData(uint8_t* data);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        if (!bluetooth_connected) bluetooth_connected = 1;

        if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '5') {
            target_direction = bluetooth_rx_data - '0';

            // ?????????
            if (target_direction == DIR_STOP) {
                target_speed = 0;
                is_turning_angle = 0;
                is_turning_u = 0;
            }

            // ??/??
            else if (target_direction == DIR_FORWARD || target_direction == DIR_BACKWARD) {
                target_speed = MAX_SPEED;
                is_turning_angle = 0;
                is_turning_u = 0;
            }

            // ?/?? - ?????????
            else if (target_direction == DIR_LEFT || target_direction == DIR_RIGHT) {
                is_turning_angle = 1;
                is_turning_u = 0;
                turning_angle_accum = 0.0f;
                turning_direction = target_direction;
                target_speed = TURN_SPEED;
                current_direction = target_direction;
                ControlMotor(current_direction, target_speed);
            }

            // ??(????180?
            else if (target_direction == DIR_UTURN) {
                is_turning_u = 1;
                is_turning_angle = 0;
                turning_angle_u_accum = 0.0f;
                current_direction = DIR_UTURN;
                target_speed = TURN_SPEED;
                ControlMotor(DIR_UTURN, target_speed);
            }
        }

        HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
    }
    else if (huart == &huart6) {
        HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
        if (lidar_dma_buffer[0] == 0xA5) lidar_process_packet = 1;
    }else if (huart->Instance == UART4)
    {
        uint8_t Res = uart4_rx_data;
        uint8_t temp;
//        HAL_UART_Transmit(&huart1, "AAAAAA", 7, 100);
        //????

        if (uart4_rx_con < 3)
        {
            if(uart4_rx_con == 0)  //????1
            {
                //????1
                if((Res>>4) == LS_HEADER1)
                {
                    uart4_rx_buf[uart4_rx_con] = Res;
                    uart4_rx_con = 1;
                }
            }
            else if(uart4_rx_con == 1) //????2
            {
                //????2
                if((Res>>4) == LS_HEADER2)
                {
                    uart4_rx_buf[uart4_rx_con] = Res;
                    uart4_rx_con = 2;
                }
                else
                {
                    uart4_rx_con = 0;
                }
            }
            else  //???????
            {
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_con = 3;

                //????
                uart4_rx_chksum = Res;
            }
        }
        else  //????
        {
            //???????
            if(uart4_rx_con < (LS_F_LEN-1))
            {
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_con++;
                uart4_rx_chksum = uart4_rx_chksum^Res;
            }
            else
            {
                //????????
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_chksum = uart4_rx_chksum^Res;

                //??
                uart4_rx_con = 0;

                //?????????
                temp = ((uint8_t)(uart4_rx_buf[1]<<4)) + (uint8_t)(uart4_rx_buf[0]&0x0F);

                //????????
                if( uart4_rx_chksum == temp)
                {
                    //????,???????
                    LS_DataHandle();
                }
            }
        }

        // ?????????
        HAL_UART_Receive_IT(&huart4, &uart4_rx_data, 1);
    }
}




void ControlMotor(uint8_t direction, uint32_t speed) {
    switch(direction) {
        case DIR_STOP:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
       case DIR_FORWARD:{
            float pwmA = PID_Compute(&pidA, fabsf(rpmA));
            float pwmB = PID_Compute(&pidB, fabsf(rpmB));
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pwmA);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pwmB);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
			 }
        case DIR_BACKWARD: {
            float pwmA = PID_Compute(&pidA, fabsf(rpmA));
            float pwmB = PID_Compute(&pidB, fabsf(rpmB));
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwmA);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)pwmB);
            break;
        }
case DIR_LEFT:
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    break;
case DIR_RIGHT:
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,  TURN_SPEED);
    break;
case DIR_UTURN:  // ?????:????,????
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);  // ????
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);  // ????
    break;
    }
}

void UpdateSpeedRamp(void) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        last_speed_update_time = current_time;

        if (target_direction != current_direction) {
            // ??/????????
            if ((target_direction == DIR_FORWARD || target_direction == DIR_BACKWARD) &&
                (current_direction == DIR_STOP || current_direction == DIR_FORWARD || current_direction == DIR_BACKWARD)) {
                current_direction = target_direction;
                target_speed = MAX_SPEED;
                ControlMotor(current_direction, target_speed);
            }
            // ??????(??/??/??)
            else if (target_direction == DIR_LEFT || target_direction == DIR_RIGHT || target_direction == DIR_UTURN) {
                target_speed = (target_direction == DIR_UTURN) ? TURN_SPEED : TURN_SPEED;
                current_direction = target_direction;
                ControlMotor(current_direction, target_speed);
            }
            // ??????
            else if (target_direction == DIR_STOP) {
                target_speed = 0;
                if (current_speed > 0) {
                    current_speed -= SPEED_STEP;
                    ControlMotor(current_direction, current_speed);
                } else {
                    current_direction = target_direction;
                    ControlMotor(current_direction, target_speed);
                }
            }
        }
        // ??????
        else if (current_direction != DIR_STOP) {
            if (current_speed < target_speed) {
                current_speed += SPEED_STEP;
            } else if (current_speed > target_speed) {
                current_speed -= SPEED_STEP;
            }
            ControlMotor(current_direction, current_speed);
        }
    }
}
void SendConnectionNotification(void) {
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, sizeof(connection_msg)-1, 1000);
        connection_announced = 1;
    }
}

void ProcessLidarData(uint8_t* data) {
    typedef struct {
        uint8_t sync_quality;
        uint16_t angle_q6;
        uint16_t distance_q2;
    } __attribute__((packed)) LidarDataPacket;

    LidarDataPacket* pkt = (LidarDataPacket*)data;
    if((pkt->sync_quality & 0x80) && (pkt->angle_q6 & 0x01)) {
        uint8_t quality = pkt->sync_quality & 0x7F;
        float angle = (pkt->angle_q6 >> 1) / 64.0f;
        float distance = pkt->distance_q2 / 4.0f;
        if(distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE && quality > 0) {
            if(fabsf(angle - lastLidarAngle) >= ANGLE_FILTER_THRESHOLD || angle < lastLidarAngle) {
                SendLidarToBluetooth(angle, distance, quality);
                lastLidarAngle = angle;
            }
        }
    }
    if(lidar_process_packet) {
        lastLidarRxTime = HAL_GetTick();
        __HAL_DMA_DISABLE(huart6.hdmarx);
        huart6.hdmarx->Instance->NDTR = DMA_BUFFER_SIZE;
        __HAL_DMA_ENABLE(huart6.hdmarx);
        lidar_process_packet = 0;
    }
}

void SendLidarToBluetooth(float angle, float distance, uint8_t quality) {
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "A:%.2f,D:%.2f,Q:%d\n", angle, distance, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
	MX_UART4_Init();
    MX_I2C1_Init();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
	HAL_UART_Receive_IT(&huart4, &uart4_rx_data, 1);
    MPU6500_Data mpu_data;
    if (MPU6500_Init(&hi2c1) != 0) {
        strcpy(uart_buf, "MPU6500 init failed!\r\n");
    } else {
        strcpy(uart_buf, "MPU6500 OK\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

    HAL_Delay(500);
    uint8_t startCmd[] = {0xA5, 0x20};
    HAL_UART_Transmit(&huart6, startCmd, sizeof(startCmd), 100);
    HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);

    while (1) {
        SendConnectionNotification();
        UpdateSpeedRamp();
			// ==== PID?????(?????????) ====
        if (current_direction == DIR_FORWARD || current_direction == DIR_BACKWARD) {
            pidA.setpoint = target_speed;
            pidB.setpoint = target_speed;
        } else {
            pidA.setpoint = 0;
            pidB.setpoint = 0;
            pidA.integral = pidB.integral = 0; // ??????
        }
		uint8_t a[100];

		sprintf(a, "angle  distance ");

		HAL_UART_Transmit(&huart4,a,sizeof(a),HAL_MAX_DELAY);///////////////////////////////////



        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);
        int32_t deltaA = encoderA - lastEncoderA;
        int32_t deltaB = encoderB - lastEncoderB;
        lastEncoderA = encoderA;
        lastEncoderB = encoderB;
        rpmA = (float)deltaA / PPR * (60.0f / (SAMPLE_TIME_MS / 1000.0f));
        rpmB = (float)deltaB / PPR * (60.0f / (SAMPLE_TIME_MS / 1000.0f));
        snprintf(uart_buf, sizeof(uart_buf), "MotorA:%.1f,RPMB:%.1f\n", rpmA, rpmB);
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        float voltage = (adc_val * 3.3f * 11.0f) / 4096.0f;

        uint32_t currentRxIndex = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        if (currentRxIndex != lidar_rxIndex) {
            uint32_t dataLength = (currentRxIndex > lidar_rxIndex) ?
                                  (currentRxIndex - lidar_rxIndex) :
                                  (DMA_BUFFER_SIZE - lidar_rxIndex + currentRxIndex);
            for (uint32_t i = 0; i < dataLength; i++) {
                uint8_t data = lidar_dma_buffer[(lidar_rxIndex + i) % DMA_BUFFER_SIZE];
                if(lidar_dataIndex == 0 && data != 0xA5) continue;
                lidar_dataBuffer[lidar_dataIndex++] = data;
                if(lidar_dataIndex >= DATA_PACKET_SIZE) {
                    ProcessLidarData(lidar_dataBuffer);
                    lidar_dataIndex = 0;
                }
            }
            lidar_rxIndex = currentRxIndex;
        }
        if(lidar_dataIndex > 0 && (HAL_GetTick() - lastLidarRxTime > LIDAR_TIMEOUT_THRESHOLD)) {
            lidar_dataIndex = 0;
        }

        // MPU6500
// MPU6500 ??
MPU6500_ReadData(&hi2c1, &mpu_data);

// ?? MPU ??
snprintf(uart_buf, sizeof(uart_buf),
    "AccX:%.2f,AccY:%.2f,AccZ:%.2f,GyroX:%.2f,GyroY:%.2f,GyroZ:%.2f\n",
    mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z,
    mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

// ??????????,???????
if (is_turning_angle) {
    float gyro_z = mpu_data.gyro_z;  // ??:?/?
    turning_angle_accum += gyro_z * (SAMPLE_TIME_MS / 1000.0f);  // ??,??:?

    // ??????(???,??)
    snprintf(uart_buf, sizeof(uart_buf), "TurningAngle:%.2f\n", turning_angle_accum);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

    float target_angle = 90.0f;
if (is_turning_u) {
    float gyro_z = mpu_data.gyro_z;  // deg/s
    turning_angle_u_accum += gyro_z * (SAMPLE_TIME_MS / 1000.0f);

    // ????????(???,??)
    snprintf(uart_buf, sizeof(uart_buf), "U-TurnAngle:%.2f\n", turning_angle_u_accum);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

    if (turning_angle_u_accum >= 180.0f) {
        is_turning_u = 0;
        target_direction = DIR_STOP;
        current_direction = DIR_STOP;
        target_speed = 0;
        ControlMotor(DIR_STOP, 0);
    }
}
    // ??????90?,????
    if ((turning_direction == DIR_LEFT && turning_angle_accum >= target_angle) ||
        (turning_direction == DIR_RIGHT && turning_angle_accum <= -target_angle)) {
        is_turning_angle = 0;
        target_direction = DIR_STOP;
        current_direction = DIR_STOP;
        target_speed = 0;
        ControlMotor(DIR_STOP, 0);
    }
}
}
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

///////////////////////////////////////////////////////////////

void LS_DataHandle(void)
{

	uint8_t i;
	float temp;

	static uint16_t cnt = 0;

	static float angle_last = 0;

	//Ã¿Ãë²É¼¯5000´Î10HZ£¬×ªÒ»È¦²É¼¯500¸öµã£¬·½±ãµ¥Æ¬»ú´¦Àí£¬2¸öµãÈ¡Ò»¸öµã
	static LaserPointTypeDef point[250];


		    // ?????????????
//    char temp_str[50];
//    HAL_UART_Transmit(&huart1, (uint8_t*)"RX Buffer: ", 11, 100);  // ????

//    for (int i = 0; i < 10; i++) {
//        // ????????????
//        sprintf(temp_str, "%02X ", uart4_rx_buf[i]);
//        HAL_UART_Transmit(&huart1, (uint8_t*)temp_str, strlen(temp_str), 100);  // ???????
//    }
//    // ???
//    HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 100);


	float angle_new = (((uint16_t)((uart4_rx_buf[3]&0x7F)<<8)) + uart4_rx_buf[2])/64.0;
	float angle_area;


//    sprintf(temp_str, "uart4_rx_buf[2]: %02X, uart4_rx_buf[3]: %02X, angle_new: %.2f\n", uart4_rx_buf[2], uart4_rx_buf[3], angle_new);
//    HAL_UART_Transmit(&huart1, (uint8_t*)temp_str, strlen(temp_str), 100);




	//ÆðÊ¼½Ç¶È´óÓÚ½áÊø½Ç¶È£¬¿ç¹ý360¡ã
	if(angle_new > angle_last)
	{
		angle_area  = (angle_new - angle_last)/20;

		for(i=0; i<20; i++)
		{
			temp = angle_new + angle_area*i;

			//¼ÆËã½Ç¶È
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}

			//¼ÆËã¾àÀë
			point[cnt+i].distance =  ((uint16_t)(uart4_rx_buf[5+i*4]<<8)) + (uint8_t)uart4_rx_buf[4+i*4];

		}
	}
	else
	{
		angle_area = (angle_new + 360 - angle_last)/20;

		for(i=0; i<20; i++)
		{

			temp = angle_new + angle_area*i;


			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}

			//¼ÆËã¾àÀë
			point[cnt+i].distance =  ((uint16_t)(uart4_rx_buf[5+i*4]<<8)) + (uint8_t)uart4_rx_buf[4+i*4];
		}

	}

	//¸³ÖµÉÏÒ»´Î²âÁ¿½Ç¶È
	angle_last = angle_new;

	//Êä³öµ÷ÊÔÊý¾Ý
	//printf("%d %d %d \r\n",cnt, point[0].angle, point[0].distance);

	//Ò»Ö¡Êý¾Ý½âÎö½áÊø
	cnt = cnt+20;

	//ÅÐ¶ÏÊÇ·ñ×ªÍäÒ»È¦£¨À×´ï×ªÒ»È¦ÓÐ250¸öµã£©

	if(cnt > 260)
	{
		//½«Êý×éµÄÊý¾Ý×ªÒÆµ½Íâ²¿Êý×éÖÐ£¬±ÜÃâ¸²¸ÇÊý¾Ý
		for(i=0; i<250; i++)
		{
			//¼ÆËã½Ç¶È
			ax_ls_point[i].angle = point[i].angle;
			ax_ls_point[i].distance = point[i].distance;
		////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////



		}

		//¸´Î»
		cnt = 0;
	}

}


/**
  * @¼ò  Êö  À×´ïÆô¶¯£¨ÃÜÊµÄ£Ê½£©
  * @²Î  Êý  ÎÞ
  * @·µ»ØÖµ	 ÎÞ
  */
void AX_LASER_Start(void)
{
	uint8_t i;

	uart4_tx_buf[0] = 0xA5;  //Ö¡Í·
	uart4_tx_buf[1] = 0x82;  //Æô¶¯É¨ÃèÃüÁî
	uart4_tx_buf[2] = 05;
	uart4_tx_buf[3] = 0;
	uart4_tx_buf[4] = 0;
	uart4_tx_buf[5] = 0;
	uart4_tx_buf[6] = 0;
	uart4_tx_buf[7] = 0;
	uart4_tx_buf[8] = 0x22;  //Ð£ÑéºÍ

//	uart4_tx_buf[0] = 0xA5;  //Ö¡Í·
//	uart4_tx_buf[1] = 0x20;  //Æô¶¯É¨ÃèÃüÁî

//
//
//		HAL_UART_Transmit(&huart4, &uart4_tx_buf[0], 1, 100);
//		HAL_UART_Transmit(&huart4, &uart4_tx_buf[1], 1, 100);
//		HAL_UART_Transmit(&huart1, &uart4_tx_buf[0], 1, 100);
//		HAL_UART_Transmit(&huart1, &uart4_tx_buf[1], 1, 100);
	if (HAL_UART_Receive_IT(&huart4, &uart4_rx_data, 1) != HAL_OK) {
      Error_Handler();
  }
  	HAL_Delay(2000);

	//²éÑ¯´«Êä·½Ê½
	for(i=0; i<9; i++)
	{

//		USART_SendData(UART4, uart4_tx_buf[i]);
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TC) != SET);


		HAL_UART_Transmit(&huart1, &uart4_tx_buf[i], 1, 100);
		HAL_UART_Transmit(&huart4, &uart4_tx_buf[i], 1, 100);
		while (HAL_UART_GetState(&huart4) == HAL_UART_STATE_BUSY_TX);
//		HAL_UART_Transmit(&huart1, "AAAAAA", 7, 100);
	}
}


/**
  * @¼ò  Êö  À×´ï¹Ø±Õ
  * @²Î  Êý  ÎÞ
  * @·µ»ØÖµ	 ÎÞ
  */
void AX_LASER_Stop(void)
{
	uint8_t i;

	uart4_tx_buf[0] = 0xA5;       //Ö¡Í·
	uart4_tx_buf[1] = 0x25;       //¹Ø±ÕÃüÁî
	uart4_tx_buf[2] = 0xA5+0x25;  //Ð£ÑéºÍ

	//²éÑ¯´«Êä·½Ê½
	for(i=0; i<3; i++)
	{
//		USART_SendData(UART4, uart4_tx_buf[i]);
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TC) != SET);


		HAL_UART_Transmit(&huart4, &uart4_tx_buf[i], 1, 100);
		while (HAL_UART_GetState(&huart4) == HAL_UART_STATE_BUSY_TX);
	}
}
