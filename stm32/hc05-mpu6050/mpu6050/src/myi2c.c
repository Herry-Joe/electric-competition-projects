#include "myi2c.h"

static void i2c_delay(void)
{
    volatile uint32_t n = I2C_DELAY_US * 10;
    while (n--);
}

/* ======== I2C1 (OLED: PB8=SCL, PB9=SDA) ======== */

static inline void SCL1_H(void) { I2C1_SCL_PORT->BSRR = I2C1_SCL_PIN; }
static inline void SCL1_L(void) { I2C1_SCL_PORT->BRR  = I2C1_SCL_PIN; }
static inline void SDA1_H(void) { I2C1_SDA_PORT->BSRR = I2C1_SDA_PIN; }
static inline void SDA1_L(void) { I2C1_SDA_PORT->BRR  = I2C1_SDA_PIN; }
static inline uint8_t SDA1_R(void) { return (I2C1_SDA_PORT->IDR & I2C1_SDA_PIN) ? 1 : 0; }

static void SDA1_OUT(void)
{ GPIO_InitTypeDef g={0}; g.Pin=I2C1_SDA_PIN; g.Mode=GPIO_MODE_OUTPUT_OD; g.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(I2C1_SDA_PORT,&g); }
static void SDA1_IN(void)
{ GPIO_InitTypeDef g={0}; g.Pin=I2C1_SDA_PIN; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_NOPULL; HAL_GPIO_Init(I2C1_SDA_PORT,&g); }

void I2C1_Start(void)  { SDA1_OUT(); SDA1_H(); SCL1_H(); i2c_delay(); SDA1_L(); i2c_delay(); SCL1_L(); }
void I2C1_Stop(void)   { SDA1_OUT(); SCL1_L(); SDA1_L(); i2c_delay(); SCL1_H(); i2c_delay(); SDA1_H(); }

uint8_t I2C1_WaitAck(void)
{ uint8_t t=0; SDA1_IN(); SDA1_H(); i2c_delay(); SCL1_H(); i2c_delay();
  while(SDA1_R()){if(++t>250){I2C1_Stop();return 1;}} SCL1_L(); return 0; }

void I2C1_WriteByte(uint8_t d)
{ uint8_t i; SDA1_OUT(); SCL1_L();
  for(i=0;i<8;i++){d&0x80?SDA1_H():SDA1_L();d<<=1;i2c_delay();SCL1_H();i2c_delay();SCL1_L();}
  i2c_delay();SCL1_H();i2c_delay();SCL1_L(); }

uint8_t I2C1_ReadByte(uint8_t ack)
{ uint8_t i,v=0; SDA1_IN();
  for(i=0;i<8;i++){SCL1_L();i2c_delay();SCL1_H();v<<=1;if(SDA1_R())v|=1;i2c_delay();}
  SCL1_L(); SDA1_OUT();
  if(ack){SDA1_L();i2c_delay();SCL1_H();i2c_delay();SCL1_L();}
  else   {SDA1_H();i2c_delay();SCL1_H();i2c_delay();SCL1_L();}
  return v; }

void I2C1_WriteReg(uint8_t addr,uint8_t reg,uint8_t data)
{ I2C1_Start();I2C1_WriteByte(addr<<1);I2C1_WaitAck();I2C1_WriteByte(reg);I2C1_WaitAck();I2C1_WriteByte(data);I2C1_WaitAck();I2C1_Stop(); }

uint8_t I2C1_ReadReg(uint8_t addr,uint8_t reg)
{ uint8_t v; I2C1_Start();I2C1_WriteByte(addr<<1);I2C1_WaitAck();I2C1_WriteByte(reg);I2C1_WaitAck();
  I2C1_Start();I2C1_WriteByte((addr<<1)|1);I2C1_WaitAck();v=I2C1_ReadByte(0);I2C1_Stop();return v; }

void I2C1_ReadBytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ uint8_t i; I2C1_Start();I2C1_WriteByte(addr<<1);I2C1_WaitAck();I2C1_WriteByte(reg);I2C1_WaitAck();
  I2C1_Start();I2C1_WriteByte((addr<<1)|1);I2C1_WaitAck();
  for(i=0;i<len;i++) buf[i]=I2C1_ReadByte(i<len-1);
  I2C1_Stop(); }

/* ======== I2C2 (MPU6050: PB10=SCL, PB11=SDA) ======== */

static inline void SCL2_H(void) { I2C2_SCL_PORT->BSRR = I2C2_SCL_PIN; }
static inline void SCL2_L(void) { I2C2_SCL_PORT->BRR  = I2C2_SCL_PIN; }
static inline void SDA2_H(void) { I2C2_SDA_PORT->BSRR = I2C2_SDA_PIN; }
static inline void SDA2_L(void) { I2C2_SDA_PORT->BRR  = I2C2_SDA_PIN; }
static inline uint8_t SDA2_R(void) { return (I2C2_SDA_PORT->IDR & I2C2_SDA_PIN) ? 1 : 0; }

/* 总线恢复: 9个时钟脉冲释放SDA */
static void I2C2_Recover(void)
{
    uint8_t i;
    SDA2_H(); SCL2_H();
    for (i = 0; i < 9; i++) {
        SCL2_L(); i2c_delay(); SCL2_H(); i2c_delay();
    }
    I2C2_Start(); I2C2_Stop();
}

void I2C2_Start(void)
{ SDA2_H(); SCL2_H(); i2c_delay(); SDA2_L(); i2c_delay(); SCL2_L(); }

void I2C2_Stop(void)
{ SCL2_L(); SDA2_L(); i2c_delay(); SCL2_H(); i2c_delay(); SDA2_H(); }

uint8_t I2C2_WaitAck(void)
{ uint8_t t=0; SDA2_H(); i2c_delay(); SCL2_H(); i2c_delay();
  while(SDA2_R()){if(++t>250){I2C2_Stop();return 1;}} SCL2_L(); return 0; }

void I2C2_WriteByte(uint8_t d)
{ uint8_t i; SCL2_L();
  for(i=0;i<8;i++){d&0x80?SDA2_H():SDA2_L();d<<=1;i2c_delay();SCL2_H();i2c_delay();SCL2_L();} }

uint8_t I2C2_ReadByte(uint8_t ack)
{ uint8_t i,v=0; SDA2_H();
  for(i=0;i<8;i++){SCL2_L();i2c_delay();SCL2_H();v<<=1;if(SDA2_R())v|=1;i2c_delay();}
  SCL2_L();
  if(ack){SDA2_H();}else{SDA2_L();}
  i2c_delay();SCL2_H();i2c_delay();SCL2_L();
  return v; }

void I2C2_WriteReg(uint8_t addr,uint8_t reg,uint8_t data)
{ I2C2_Start();I2C2_WriteByte(addr<<1);I2C2_WaitAck();I2C2_WriteByte(reg);I2C2_WaitAck();I2C2_WriteByte(data);I2C2_WaitAck();I2C2_Stop(); }

uint8_t I2C2_ReadReg(uint8_t addr,uint8_t reg)
{ uint8_t v;
  I2C2_Start();I2C2_WriteByte(addr<<1);I2C2_WaitAck();I2C2_WriteByte(reg);I2C2_WaitAck();
  I2C2_Start();I2C2_WriteByte((addr<<1)|1);I2C2_WaitAck();v=I2C2_ReadByte(1);I2C2_Stop();return v; }

void I2C2_ReadBytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ uint8_t i;
  I2C2_Start();I2C2_WriteByte(addr<<1);I2C2_WaitAck();I2C2_WriteByte(reg);I2C2_WaitAck();
  I2C2_Start();I2C2_WriteByte((addr<<1)|1);I2C2_WaitAck();
  for(i=0;i<len;i++) buf[i]=I2C2_ReadByte(i==len-1);
  I2C2_Stop(); }

/* ======== Init I2C GPIO ======== */

void I2C_InitAll(void)
{
    GPIO_InitTypeDef g={0};

    /* Enable both GPIOA and GPIOB clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* I2C2: PB10=SCL, PB11=SDA (open-drain) - MPU6050 */
    g.Pin=I2C2_SCL_PIN|I2C2_SDA_PIN; g.Mode=GPIO_MODE_OUTPUT_OD; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C2_SDA_PORT,&g); SCL2_H(); SDA2_H();
}

