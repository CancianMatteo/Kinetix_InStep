#include <rtrobot_lsm6dsv320x.h>

// SPI
// 5.0V  --  VCC
// GND --  GND
// 13  --  SCK
// 12  --  SDO
// 11  --  SDI
// 10  --  CS

// I2C
//  Connect Vin to 3-5VDC
//  Connect GND to ground
//  Connect SCL to I2C clock pin (A5 on UNO)
//  Connect SDA to I2C data pin (A4 on UNO)

RTROBOT_LSM6DSV320X lsm;
#define cs_pin 10

void setup()
{
    delay(2000);
    Serial.begin(115200);
    while (lsm.begin(RTROBOT_INTERFACE_BUS_I2C, LSM6DSV320X_I2C_ADD_H) == false)
    {
        Serial.println("LSM6DS3TRC not found!");
        delay(500);
    }
    Serial.println("LSM6DSV320X found!");

    lsm6dsv320x_reset_t rst;

    /* Restore default configuration */
    lsm.reset_set(LSM6DSV320X_RESTORE_CTRL_REGS);
    do
    {
        lsm.reset_get(&rst);
    } while (rst != LSM6DSV320X_READY);

    /* Enable Block Data Update */
    lsm.block_data_update_set(PROPERTY_ENABLE);
    /* Set Output Data Rate.
     * Selected data rate have to be equal or greater with respect
     * with MLC data rate.
     */
    lsm.xl_data_rate_set(LSM6DSV320X_ODR_AT_15Hz);
    lsm.hg_xl_data_rate_set(LSM6DSV320X_HG_XL_ODR_AT_480Hz, 1);
    lsm.gy_data_rate_set(LSM6DSV320X_ODR_AT_60Hz);

    /* Set full scale */
    lsm.xl_full_scale_set(LSM6DSV320X_16g);
    lsm.hg_xl_full_scale_set(LSM6DSV320X_320g);
    lsm.gy_full_scale_set(LSM6DSV320X_4000dps);

    /* Configure filtering chain */
    lsm.filt_settling_mask.drdy = PROPERTY_ENABLE;
    lsm.filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    lsm.filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm.filt_settling_mask_set(lsm.filt_settling_mask);
    lsm.filt_gy_lp1_set(PROPERTY_ENABLE);
    lsm.filt_gy_lp1_bandwidth_set(LSM6DSV320X_GY_ULTRA_LIGHT);
    lsm.filt_xl_lp2_set(PROPERTY_ENABLE);
    lsm.filt_xl_lp2_bandwidth_set(LSM6DSV320X_XL_STRONG);
}

void loop()
{

    int16_t data_raw_motion[3];
    int16_t data_raw_temperature;
    float_t acceleration_mg[3];
    float_t angular_rate_mdps[3];
    float_t temperature_degC;

    lsm6dsv320x_data_ready_t drdy;

    /* Read output only if new xl value is available */
    lsm.flag_data_ready_get(&drdy);
    if (drdy.drdy_xl)
    {
        /* Read acceleration field data */
        lsm.acceleration_raw_get(data_raw_motion);

        acceleration_mg[0] = lsm.from_fs16_to_mg(data_raw_motion[0]);
        acceleration_mg[1] = lsm.from_fs16_to_mg(data_raw_motion[1]);
        acceleration_mg[2] = lsm.from_fs16_to_mg(data_raw_motion[2]);
        Serial.print("lg xl [mg]: ");
        Serial.print(acceleration_mg[0], 2);
        Serial.print("\t");
        Serial.print(acceleration_mg[1], 2);
        Serial.print("\t");
        Serial.println(acceleration_mg[2], 2);
    }

    if (drdy.drdy_hgxl)
    {
        /* Read acceleration field data */
        lsm.hg_acceleration_raw_get(data_raw_motion);
        acceleration_mg[0] = lsm.from_fs320_to_mg(data_raw_motion[0]);
        acceleration_mg[1] = lsm.from_fs320_to_mg(data_raw_motion[1]);
        acceleration_mg[2] = lsm.from_fs320_to_mg(data_raw_motion[2]);
        Serial.print("hg xl [mg]: ");
        Serial.print(acceleration_mg[0], 2);
        Serial.print("\t");
        Serial.print(acceleration_mg[1], 2);
        Serial.print("\t");
        Serial.println(acceleration_mg[2], 2);
    }

    /* Read output only if new xl value is available */
    if (drdy.drdy_gy)
    {
        /* Read angular rate field data */
        lsm.angular_rate_raw_get(data_raw_motion);
        angular_rate_mdps[0] = lsm.from_fs4000_to_mdps(data_raw_motion[0]);
        angular_rate_mdps[1] = lsm.from_fs4000_to_mdps(data_raw_motion[1]);
        angular_rate_mdps[2] = lsm.from_fs4000_to_mdps(data_raw_motion[2]);
        Serial.print("gyro [mdps]: ");
        Serial.print(angular_rate_mdps[0], 2);
        Serial.print("\t");
        Serial.print(angular_rate_mdps[1], 2);
        Serial.print("\t");
        Serial.println(angular_rate_mdps[2], 2);
    }

    if (drdy.drdy_temp)
    {
        /* Read temperature data */
        lsm.temperature_raw_get(&data_raw_temperature);
        temperature_degC = lsm6dsv320x_from_lsb_to_celsius(data_raw_temperature);
        Serial.print("Temperature [°C]: ");
        Serial.println(temperature_degC, 2);
        Serial.println();
    }

    delay(1000);
}
