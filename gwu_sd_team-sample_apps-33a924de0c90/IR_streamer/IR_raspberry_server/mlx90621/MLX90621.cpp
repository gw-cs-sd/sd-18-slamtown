#include <MLX90621.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <bcm2835.h>


bool MLX90621::ReadConstCoeff() {

    kmlx.resolution = (EEPROM[EEPROM_CFG_REG_L] & 0x30) >> 4;
    kmlx.ta0 = 25.0;

    kmlx.alpha0 = ((EEPROM[EEPROM_ALPHA0_H] << 8) 
                  | EEPROM[EEPROM_ALPHA0_L]);
    short alpha0_scale = EEPROM[EEPROM_ALPHA0_SCALE];
    kmlx.alpha0 /= pow(2, alpha0_scale);
    
    kmlx.delta_alpha_scale = EEPROM[EEPROM_dALPHA_SCALE];
    kmlx.KsTa = (short)((EEPROM[EEPROM_KSTA_H] << 8) 
                       | EEPROM[EEPROM_KSTA_L]);
    kmlx.KsTa /= pow(2, 20);    
    kmlx.alpha_cp = ((EEPROM[EEPROM_ALPHA_CP_H] << 8) 
                    | EEPROM[EEPROM_ALPHA_CP_L]);
    kmlx.alpha_cp /= (pow(2, alpha0_scale + 3 - kmlx.resolution));

    kmlx.tgc = (char)(EEPROM[EEPROM_TGC]);
    kmlx.tgc /= 32.0;

    kmlx.dai_scale = EEPROM[EEPROM_A_B_SCALING] >> 4;
    kmlx.bi_scale = EEPROM[EEPROM_A_B_SCALING] & 0x0F;
    kmlx.a_common = (short)((EEPROM[EEPROM_A_COMMON_H] << 8)
                           | EEPROM[EEPROM_A_COMMON_L]);
    if (kmlx.a_common > 32767) {
      printf(" [Error] A_common not negative! \n");
    }

    float bcp_raw = (char)(EEPROM[EEPROM_B_CP]);
    if (bcp_raw > 127) {
        bcp_raw = bcp_raw - 256;
    }
    kmlx.b_cp = bcp_raw / 
            (pow(2, kmlx.bi_scale + 3 - kmlx.resolution));
    kmlx.a_cp = (short)(EEPROM[EEPROM_A_CP_H] << 8 
                      | EEPROM[EEPROM_A_CP_L]);
    if (kmlx.a_cp > 32767) { 
	printf(" [Error] A_common not negative! \n");
    }
    kmlx.a_cp /= pow(2, 3 - kmlx.resolution);

    kmlx.emisivity = 0.98;//((EEPROM[EEPROM_EMISIVITY_H] << 8) 
                     //| EEPROM[EEPROM_EMISIVITY_L]) / 32768.0;

    return true;
}


float MLX90621::ComputeTo() {
   
    // Read sensor pixel compensation
    float avg_to = 0;
    const size_t start_x = 1;
    const size_t roi_width = 16 - 2 * start_x; 
    const size_t start_y = 0;
    const size_t roi_height = 4 - 2 * start_y;
    for (size_t i = start_y; i < start_y + roi_height; ++i) {
        for (size_t j = start_x; j < start_x + roi_width; ++j) {
        
        size_t x = i * 16 + j;

        // Compute alpha compensated
        float delta_alpha = EEPROM[EEPROM_dALPHA_START + x];
        float alpha = (kmlx.alpha0 + delta_alpha 
                / pow(2, kmlx.delta_alpha_scale)) 
                / pow(2, 3 - kmlx.resolution);
        float alpha_comp = (1 + kmlx.KsTa * (Ta - kmlx.ta0)) 
                * (alpha - kmlx.tgc * kmlx.alpha_cp);

        // Compute vir offset compensated
        float vir = (short)((ir_pixels[x*2+1] << 8) 
                           | ir_pixels[x*2]);
        short dai = EEPROM[x];
        float ai = (kmlx.a_common + dai * pow(2, kmlx.dai_scale)) 
                / pow(2, 3 - kmlx.resolution);
        float bi = (char)(EEPROM[EEPROM_BI_START + x]);
	if (bi > 127) {
		bi = bi - 256;
	}
        bi = bi / (pow(2, kmlx.bi_scale + 3 - kmlx.resolution));
        float vir_off_comp = vir - ( ai + bi * (Ta - kmlx.ta0));

        // Compute vcp offset compensated
        float vcp_off_comp = (float)kmlx.vcp 
                - (kmlx.a_cp + kmlx.b_cp * (Ta - kmlx.ta0));

        // Compute vir compensated
        float vir_tgc_comp = vir_off_comp - kmlx.tgc * vcp_off_comp;
        float vir_compensated = vir_tgc_comp / kmlx.emisivity;

        // Compute To
        float magic_number = 0.0;
        frame_temp[x] = pow((vir_compensated / alpha_comp 
                + pow((Ta + 273.15), 4)), 1.0/4.0) 
                - 273.15 + magic_number;
        avg_to += frame_temp[x];
        }
    }

    avg_to /= (roi_width * roi_height);
    float std_dev = 0.0;
    for (size_t i = start_y; i < start_y + roi_height; ++i) {
        for (size_t j = start_x; j < start_x + roi_width; ++j) {
        
            size_t x = i * 16 + j;
            std_dev += (avg_to - frame_temp[x]) * (avg_to - frame_temp[x]);
        }
    }
    std_dev /= (roi_width * roi_height);
    if (std_dev > 1.7*1.7) {
        return -101.0;  // invalid To (random number, out of To values range)
    }

    return avg_to;
}

float MLX90621::ComputeToWithRead() {
	
    float a_ij[64], b_ij[64], alpha_ij[64];
    short cpix = kmlx.vcp;
    
    // Calculate variables from EEPROM
    short resolution = (EEPROM[EEPROM_CFG_REG_L] & 0x30) >> 4;

    float emissivity = (256 * EEPROM[EEPROM_EMISIVITY_H] 
            + EEPROM[EEPROM_EMISIVITY_L]) / 32768.0;

    short a_common = (short) 256 * EEPROM[EEPROM_A_COMMON_H] 
            + EEPROM[EEPROM_A_COMMON_L];
    if (a_common >= 32768)
        a_common -= 65536;

    float alpha_cp = (256 * EEPROM[EEPROM_ALPHA_CP_H] 
            + EEPROM[EEPROM_ALPHA_CP_L])
            / (pow(2, EEPROM[EEPROM_ALPHA0_SCALE]) 
            * pow(2, (3 - resolution)));

    short a_i_scale = (short) (EEPROM[EEPROM_A_B_SCALING] & 0xF0) >> 4;
    short b_i_scale = (short) EEPROM[EEPROM_A_B_SCALING] & 0x0F;

    float a_cp = (float) 256 * EEPROM[EEPROM_A_CP_H] 
            + EEPROM[EEPROM_A_CP_L];
    if (a_cp >= 32768.0)
        a_cp -= 65536.0;
    a_cp /= pow(2, (3 - resolution));

    float b_cp = (float) EEPROM[EEPROM_B_CP];
    if (b_cp > 127.0)
        b_cp -= 256.0;
    b_cp /= (pow(2, b_i_scale) * pow(2, (3 - resolution)));

    float tgc = (float) EEPROM[EEPROM_TGC];
    if (tgc > 127.0)
        tgc -= 256.0;
    tgc /= 32.0;

    float v_cp_off_comp = (float) cpix - (a_cp + b_cp 
            * (Ta - 25.0));
    float v_ir_off_comp, v_ir_tgc_comp, v_ir_norm, v_ir_comp;
    float temperatures = 0;
    int i;
    for (i = 0; i < 64; i++) {
        a_ij[i] = ((float) a_common + EEPROM[i] 
                * pow(2, a_i_scale))
                / pow(2, (3 - resolution));
        b_ij[i] = EEPROM[0x40 + i];
        if (b_ij[i] > 127)
            b_ij[i] -= 256;
        b_ij[i] = b_ij[i] / (pow(2, b_i_scale) 
                * pow(2, (3 - resolution)));
        short vir = (short)(( ir_pixels[i*2+1] << 8 ) 
                | ir_pixels[i*2]);
        v_ir_off_comp = vir - (a_ij[i] + b_ij[i] * (Ta - 25.0));
        v_ir_tgc_comp = v_ir_off_comp - tgc * v_cp_off_comp;
        alpha_ij[i] = ((256 * EEPROM[EEPROM_ALPHA0_H] 
                + EEPROM[EEPROM_ALPHA0_L]) 
                / pow(2, EEPROM[EEPROM_ALPHA0_SCALE]));                              
        alpha_ij[i] += (EEPROM[0x80 + i] 
                / pow(2, EEPROM[EEPROM_dALPHA_SCALE]));
        alpha_ij[i] = alpha_ij[i] / pow(2, 3 - resolution);                                 
        v_ir_norm = v_ir_tgc_comp / (alpha_ij[i] - tgc * alpha_cp);
        v_ir_comp = v_ir_norm / emissivity;
        temperatures += exp(
                (log((v_ir_comp + pow((Ta + 273.15), 4))) / 4.0))
                - 273.15;
    }
    temperatures /= NB_PIXELS;
    return temperatures;
}


float MLX90621::GetTo() {

    // Check POR flag
    if (!CheckPOR()) {
        printf(" [ERROR:main] POR flag clear --> Re-init \n");
        Init();
        return WRONG_T;
    }

    // Read IR values and metadata
    if (!ReadFrame()) {
        return WRONG_T;
    }
    ReadCP(&kmlx.vcp);
    Ta = ComputeTa();
    if (WRONG_T == Ta) {
        printf(" [ERROR:main] Wrong Ta value --> Re-init \n");
        Init();
        return WRONG_T;
    }

    // Compute object temperature
    To = ComputeTo();
    //float To_c = ComputeToWithRead();  
    //printf(" AVG_To %.3f(%.3f)   Ta %.3f \n", To, To_c, Ta);

    return To;
}

// Deinit IR sensor() 
bool MLX90621::Deinit() {

    // Close I2C communication
    bcm2835_i2c_end();
    bcm2835_close();
    
    return true;
}


// Init IR sensor
bool MLX90621::Init() {

    // Init I2C communication
    bcm2835_close();
    if (!bcm2835_init()) {
        printf(" [ERROR:mlx_init] Unable to init bcm2835. \n");
        return false;
    }
    bcm2835_i2c_begin();
    bcm2835_i2c_set_baudrate(100000);

    // Wait >5ms
    usleep(6000);

    // Read whole EEPROM
    if (!ReadEEPROM()) {
        printf(" [ERROR:mlx_init] Unable to read EEPROM. \n");
        return false;
    }
    // MLX90621 constants
    ReadConstCoeff();


    // Write trim
    if (!WriteTrim(EEPROM[EEPROM_TRIM_VALUE], 0x00)) {
        printf(" [ERROR:mlx_init] Unable to write trim. \n");
        return false;
    }

    // Write config register
    unsigned char lsb, msb;
    lsb = EEPROM[EEPROM_CFG_REG_L];
    msb = EEPROM[EEPROM_CFG_REG_H] | RAM_CFG_POR_MASK;
    if (!WriteConfig(lsb, msb)) {
        printf(" [ERROR:mlx_init] Unable to write config register. \n");
        return false;
    }
    //printf(" Write config %d - %d \n", EEPROM[0xF5], EEPROM[0xF6]);

    // Set read frequency
    WriteRefreshRate( 64 );

    // Check POR flag
    if (!CheckPOR()) {
        printf(" [ERROR] POR flag cleared during init. \n");
        return false;
    }

    PINFO(" [INFO:mlx_init] Init MLX succeed. \n");
    return true;
}


// Reads the whole EEPROM
bool MLX90621::ReadEEPROM() {

    byte read_eeprom[] = {
        EEPROM_READ_CMD
    };

    bcm2835_i2c_setSlaveAddress(EEPROM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write_read_rs((char *)&read_eeprom, 1,
                                  (char *)&EEPROM, EEPROM_SIZE)) {
    
        PINFO("\n [INFO:mlx_read_EEPROM] Read EEPROM succeed.");
        return true;
    }

    printf(" [ERROR:mlx_read_EEPROM] Unable to read EEPROM. \n");
    return false;
}


// Writes device configuration value
bool MLX90621::WriteConfig(byte lsb, byte msb) {

    byte write_config[] = {
        RAM_WRITE_CFG_CMD,
        (byte)(lsb - 0x55),
        lsb,
        (byte)(msb - 0x55),
        msb
    };

    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write((const char*)&write_config, 5)) {

        PINFO("\n [INFO:mlx_write_config] Write configuration register succeed.");
        return true;
    }
    
    printf(" [ERROR:mlx_write_config] Unable to write configuration register. \n");
    return false;
}


// Reads configuration register
bool MLX90621::ReadConfig(byte *lsb, byte *msb) {

    byte config[2];
    byte read_config[] = {
        RAM_READ_CMD,
        RAM_CFG_ADDRESS,
        0x00, // address step
        0x01  // number of reads
    };

    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write_read_rs((char*)&read_config, 4,
                                  (char*)&config, 2)) {
 
        PINFO("\n [INFO:mlx_read_config] Read configuration register succeed.");
        *lsb = config[0];
        *msb = config[1];        
        return true;
    }

    printf(" [ERROR:mlx_read_config] Unable to read configuration register. \n");
    return false;
}


// Writes the oscillator trimming value
bool MLX90621::WriteTrim(byte lsb, byte msb) {

    char write_trim[] = {
        RAM_WRITE_TRIM_CMD,
        (byte)(lsb - 0xAA),
        lsb,
        (byte)(msb - 0xAA),
        msb
    };
    
    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write((char *)&write_trim, 5)) {

        PINFO("\n [INFO:mlx_write_trim] Write trim register succeed.");
        return true;
    }

    printf(" [ERROR:mlx_write_trim] Unable to write trim register. \n");    
    return false;
}


// Reads oscillator trimming register
bool MLX90621::ReadTrim(byte *lsb, byte *msb) {

    byte trim_bytes[2];
    byte read_trim[] = {
        RAM_READ_CMD,
        RAM_TRIM_ADDRESS,
        0x00, // address step
        0x01  // number of reads
    };
    
    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write_read_rs((char*)&read_trim, 4,
                                  (char*)&trim_bytes, 2)) {

        PINFO("\n [INFO:mlx_read_trim] Read trim register succeed.");
        *lsb = trim_bytes[0];
        *msb = trim_bytes[1]; 
        return true;
    }

    printf(" [ERROR:mlx_read_trim] Unable to read trim register. \n");    
    return false;
}


// Returns POR/Brown-out flag
bool MLX90621::CheckPOR() {

    byte config_lsb, config_msb;
    ReadConfig(&config_lsb, &config_msb);

    if ((config_msb & RAM_CFG_POR_MASK) == RAM_CFG_POR_MASK) {

        PINFO("\n [INFO:mlx_por] POR flag set.");
        return true;
    }
    
    PINFO("\n [INFO:mlx_por] POR flag clear.");
    return false;
}


// Sets IR Refresh rate
bool MLX90621::WriteRefreshRate(int hz) {

    byte rate_bits;
    switch (hz) {
        case 512:
            rate_bits = RAM_CFG_REFRESH_512HZ;
            break;
        case 256:
            rate_bits = RAM_CFG_REFRESH_256HZ;
            break;
        case 128:
            rate_bits = RAM_CFG_REFRESH_128HZ;
            break;
        case 64:
            rate_bits = RAM_CFG_REFRESH_64HZ;
            break;
        case 32:
            rate_bits = RAM_CFG_REFRESH_32HZ;
            break;
        case 16:
            rate_bits = RAM_CFG_REFRESH_16HZ;
            break;
        case 8:
            rate_bits = RAM_CFG_REFRESH_8HZ;
            break;
        case 4:
            rate_bits = RAM_CFG_REFRESH_4HZ;
            break;
        case 2:
            rate_bits = RAM_CFG_REFRESH_2HZ;
            break;
        case 1:
            rate_bits = RAM_CFG_REFRESH_1HZ; // default
            break;
        case 0:
            rate_bits = RAM_CFG_REFRESH_0HZ; // 0.5 Hz
            break;
        default:
            rate_bits = RAM_CFG_REFRESH_1HZ;
    }

    unsigned char config_lsb, config_msb;
    if (!ReadConfig(&config_lsb, &config_msb)) {
        printf(" [ERROR:mlx_set_refresh_hz] Unable to read configuration register. \n");
        return false;
    }

    config_lsb = (config_lsb & 0xF0) | rate_bits;
    if (!WriteConfig(config_lsb, config_msb)) {
        printf(" [ERROR:mlx_set_refresh_hz] Unable to write configuration register. \n");
        return false;
    }

    PINFO("\n [INFO:mlx_set_refresh_hz] New refresh rate setted to %d.", hz);
    return true;
}


// Reads PTAT (Proportional To Absolute Temperature)
bool MLX90621::ReadPTAT(short *ptat) {

    byte ptat_bytes[2];
    byte read_ptat[] = {
        RAM_READ_CMD,
        RAM_PTAT_ADDRESS,
        0x00, // address step
        0x01  // number of reads
    };

    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write_read_rs((char*)&read_ptat, 4,
                                  (char*)&ptat_bytes, 2)) {

        *ptat = (ptat_bytes[1] << 8) | ptat_bytes[0];
        PINFO("\n [INFO:mlx_ptat] PTAT read successful: %d.", ptat);
        return true;
    }

    printf(" [ERROR:mlx_ptat] Unable to read PTAT value. \n"); 
    return false;
}


// Compensation pixel read
bool MLX90621::ReadCP(short *cp) {

    byte VCP_BYTES[2];
    byte compensation_pixel_read[] = {
        RAM_READ_CMD,
        RAM_CP_ADDRESS,
        0x00, // address step
        0x01  // number of reads
    };

    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write_read_rs((char*)&compensation_pixel_read, 4,
                                  (char*)&VCP_BYTES, 2)) {

        *cp = (short)((VCP_BYTES[1] << 8) | VCP_BYTES[0]);
        PINFO("\n [INFO:mlx_pc] CP read successful: %d.", cp);
        return true;
    }

    printf(" [ERROR:mlx_pc] Unable to read PC value. \n"); 
    return false;
}


// Computation of absolute chip temperature
float MLX90621::ComputeTa() {

    short ptat;
    if (!ReadPTAT(&ptat)) {
        return WRONG_T;
    }
    short vth_scale = (EEPROM[EEPROM_RESOLUTION] & 
                       EEPROM_RESOLUTION_MASK) >> 4;
    float true_vth_scale = pow(2, 3-vth_scale);
    float vth = (short)((EEPROM[EEPROM_VTH_H] << 8) | 
                         EEPROM[EEPROM_VTH_L]) / true_vth_scale;
    if (vth * true_vth_scale > 32767) {
        PINFO("\n [WARNING:mlx_Ta] VTH is too high !");
        return WRONG_T;
    }

    short kt1_scale = EEPROM[EEPROM_KT_SCALE] >> 4;
    float true_kt1_scale = pow(2, kt1_scale + 3 - vth_scale);
    float kt1 = (short)((EEPROM[EEPROM_KT1_H] << 8) | 
                         EEPROM[EEPROM_KT1_L]) / true_kt1_scale;
    if (kt1 * true_kt1_scale > 32767) {
        PINFO("\n [WARNING: mlx_Ta] kt1 is too high !");
        return WRONG_T;
    }
    
    short kt2_scale = (EEPROM[EEPROM_KT_SCALE] & 0x0F) + 10;
    float true_kt2_scale = pow(2, kt2_scale + 3 - vth_scale);
    float kt2 = (short)((EEPROM[EEPROM_KT2_H] << 8) | 
                         EEPROM[EEPROM_KT2_L]) / true_kt2_scale;
    if (kt2 * true_kt2_scale > 32767) {
        PINFO("\n [WARNING:mlx_Ta] kt2 is too high !");
        return WRONG_T;
    }

    float ta = ((-kt1 + sqrt(kt1*kt1 - 4.0 * kt2 * (vth - (float)ptat)))
               / (2.0 * kt2) ) + 25.0;
    if (ta > NORMAL_TA_H || ta < NORMAL_TA_L) {
        PINFO("\n [WARNING:mlx_Ta] Ta(%f) is too high or too low!", ta);
        return WRONG_T;
    }
    
    // Smooth the Ta based on the last few readings
    static float smooth_ta = 0;
    smooth_ta = 0.3 * smooth_ta + 0.7 * ta;
    PINFO("\n [INFO:mlx_Ta] Ta read successful: True %f - Smooth %f.", ta, smooth_ta);
    return smooth_ta;

/*  int16_t k_t1_scale, k_t2_scale, resolution;
    float k_t1, k_t2, v_th;
    resolution = (EEPROM[EEPROM_RESOLUTION] & 
                  EEPROM_RESOLUTION_MASK) >> 4;
    k_t1_scale = (int16_t) (EEPROM[EEPROM_KT_SCALE] & 0xF0) >> 4;
    k_t2_scale = (int16_t) (EEPROM[EEPROM_KT_SCALE] & 0x0F) + 10;
    v_th = (float) 256 * EEPROM[EEPROM_VTH_H] + EEPROM[EEPROM_VTH_L];
    if (v_th >= 32768.0)
        v_th -= 65536.0;
    v_th = v_th / pow(2, (3 - resolution));
    k_t1 = (float) 256 * EEPROM[EEPROM_KT1_H] + EEPROM[EEPROM_KT1_L];
    if (k_t1 >= 32768.0)
        k_t1 -= 65536.0;
    k_t1 /= (pow(2, k_t1_scale) * pow(2, (3 - resolution)));
    k_t2 = (float) 256 * EEPROM[EEPROM_KT2_H] + EEPROM[EEPROM_KT2_L];
    if (k_t2 >= 32768.0)
        k_t2 -= 65536.0;
    k_t2 /= (pow(2, k_t2_scale) * pow(2, (3 - resolution)));
    float Tambient = ((-k_t1 + sqrt(k_t1*k_t1 - (4 * k_t2 * (v_th - (float) ptat))))
                     / (2 * k_t2)) + 25.0;
 
    return Tambient;*/
}


// Read a new IR frame
bool MLX90621::ReadFrame() {

    byte ir_whole_frame_read[] = {
        RAM_READ_CMD,
        RAM_IR_DATA_ADDRESS,
        0x01, // address step
        0x40  // number of reads
    };

    bcm2835_i2c_setSlaveAddress(RAM_I2C_ADDRESS);
    if (BCM2835_I2C_REASON_OK ==
        bcm2835_i2c_write_read_rs((char*)&ir_whole_frame_read, 4,
                                  (char*)&ir_pixels, FRAME_SIZE)) {

	PINFO("\n [INFO:mlx_IR_read] IR values read successful.");
        return true;
    }

    printf(" [ERROR:mlx_IR_read] Unable to read IR values from RAM. \n"); 
    return false;
}
