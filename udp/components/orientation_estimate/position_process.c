#include <position_process.h>

const static float alpha = time_constant/(time_constant+period);

void orientation_estimation(raw_data metacarpo,raw_data proximal,Glove* glove,int i) {

	/* PHI	*/
	glove->fingers[i].medial.phi =acos(proximal.accelz/(sqrt(pow(proximal.accelx,2)+pow(proximal.accely,2)+
		pow(proximal.accelz,2))))*(1-alpha) + (glove->fingers[i].proximal.phi + proximal.gyrox * period ) * alpha ;

	glove->fingers[i].proximal.phi = acos(metacarpo.accelz/(sqrt(pow(metacarpo.accelx,2)+pow(metacarpo.accely,2)+
		pow(metacarpo.accelz,2))))*(1-alpha) + (glove->fingers[i].medial.phi + metacarpo.gyrox * period ) * alpha ;
	
	/* THETA */
	if (metacarpo.accelx==0) glove->fingers[i].proximal.theta = 0;
	else glove->fingers[i].proximal.theta = atan(metacarpo.accely/-metacarpo.accelx)*(1-alpha) + 
			 alpha * (glove->fingers[i].proximal.theta + metacarpo.gyroz * period);

	if (proximal.accelx==0)	glove->fingers[i].medial.theta = 0;
	else glove->fingers[i].medial.theta = atan(proximal.accely/-proximal.accelx)*(1-alpha) +
			 alpha * (glove->fingers[i].medial.theta + proximal.gyroz * period);

	glove->fingers[i].proximal.phi = glove->fingers[i].proximal.phi - glove -> frame_reference.phi;
	glove->fingers[i].medial.phi   = glove->fingers[i].medial.phi - glove->fingers[i].proximal.phi - glove -> frame_reference.phi    ;

	glove->fingers[i].proximal.theta = glove -> fingers[i].proximal.theta - glove -> frame_reference.theta ;
	glove->fingers[i].medial.theta   = glove -> fingers[i].medial.theta - glove -> fingers[i].proximal.theta - glove -> frame_reference.theta;
}

void reference_frame_orientation(raw_data reference,Glove* glove){

	/* PHI	*/
	glove->frame_reference.phi = acos(-reference.accelz/(sqrt(pow(reference.accelx,2)+
		pow(reference.accely,2)+pow(reference.accelz,2))))*(1-alpha) + 
		(glove->frame_reference.phi + (-reference.gyrox) * period ) * alpha;

	/* THETA */
	if (reference.accely==0) glove->frame_reference.theta = 0;
	else glove->frame_reference.theta = atan(-reference.accelx/reference.accely)*(1-alpha) + 
			 alpha * (glove->frame_reference.theta + (-reference.gyroz) * period);
}

void calibration(Glove* glove){
	int aux = 1;
	int ret, ret1;
	int16_t buffer[7];
	uint8_t sensor[14];
	uint8_t sensor2[14];

	memset(sensor,0,14);
	memset(sensor2,0,14);
	memset(buffer,0,14);

	/* Mux CONFIG*/
  mux_selector_config();
  gpio_set_level(pinA, 0);/* Mux initial state*/
  gpio_set_level(pinB, 1);

	while (aux < 2 ) {

		ret  = i2c_master_read_slave(0, SLAVE1_ADD,START_READ_ADD,sensor, 14);
		ret1 = i2c_master_read_slave(0, SLAVE2_ADD,START_READ_ADD,sensor2, 14);

		if (ret == ESP_ERR_TIMEOUT || ret1 == ESP_ERR_TIMEOUT) {
			ESP_LOGE(TAG, "I2C Timeout");
		} 
		else if (ret == ESP_OK && ret1 == ESP_OK) {
			/* Ref*/
			buffer[0] = (int16_t)((sensor[0]  << 8) | sensor[1]);		/* ACCEL X */
			buffer[1] = (int16_t)((sensor[2]  << 8) | sensor[3]);		/* ACCEL y */
			buffer[2] = (int16_t)((sensor[4]  << 8) | sensor[5]);		/* ACCEL z */
			buffer[3] = (int16_t)((sensor2[0] << 8) | sensor2[1]);	/* ACCEL X */
			buffer[4] = (int16_t)((sensor2[2] << 8) | sensor2[3]);	/* ACCEL Y */
			buffer[5] = (int16_t)((sensor2[4] << 8) | sensor2[5]);	/* ACCEL Z */

			/* Pressão através do potenciômetro*/
			buffer[6] = adc_read(aux,adc_chars);
			buffer[6] = buffer[6]/(Vinput - buffer[6])*R;


			/* PHI	*/
			glove->fingers[aux].proximal.phi = acos(buffer[2]/(sqrt(pow(buffer[0],2)+pow(buffer[1],2)+pow(buffer[2],2))));
			glove->fingers[aux].medial.phi = acos(buffer[5]/(sqrt(pow(buffer[3],2)+pow(buffer[4],2)+pow(buffer[5],2))));

			/* THETA */
			if (buffer[1]==0) glove->fingers[aux].proximal.theta = 0;
			else glove->fingers[aux].proximal.theta = atan(buffer[0]/buffer[1]);

			if (buffer[4]==0) glove->fingers[aux].medial.theta = 0;
			else glove->fingers[aux].medial.theta = atan(buffer[3]/buffer[4]);
			}
			aux++;
		}
}

void initialization(Glove* glove){

	uint8_t addr=1; 
	/* aux variable for counting*/
	esp_err_t error_setting[2]={ESP_OK,ESP_OK}; 

	/* Mux CONFIG*/
  mux_selector_config();

  /* Mux initial state*/
  gpio_set_level(pinA, 0);
  gpio_set_level(pinB, 1);

	glove->fingers[0].medial.theta=0;
	glove->fingers[0].medial.phi=0;
	glove->fingers[0].proximal.theta=0;
	glove->fingers[0].proximal.phi=0;
	glove->fingers[0].pressure=0;
	glove->fingers[1].medial.theta=0;
	glove->fingers[1].medial.phi=0;
	glove->fingers[1].proximal.theta=0;
	glove->fingers[1].proximal.phi=0;
	glove->fingers[1].pressure=0;
	glove->fingers[2].medial.theta=0;
	glove->fingers[2].medial.phi=0;
	glove->fingers[2].proximal.theta=0;
	glove->fingers[2].proximal.phi=0;
	glove->fingers[2].pressure=0;
	glove->fingers[3].medial.theta=0;
	glove->fingers[3].medial.phi=0;
	glove->fingers[3].proximal.theta=0;
	glove->fingers[3].proximal.phi=0;
	glove->fingers[3].pressure=0;
	glove->fingers[4].proximal.theta=0;
	glove->fingers[4].proximal.phi=0;
	glove->frame_reference.theta=0;
	glove->frame_reference.phi=0;
	glove->fingers[4].pressure=0;
	
	while(addr < 2){
		gpio_set_level(pinA, (addr & 2) >> 1);
  	gpio_set_level(pinB, addr & 1);
		error_setting[0]  = i2c_imu_setup(0,SLAVE1_ADD); 
		error_setting[1]  = i2c_imu_setup(0,SLAVE2_ADD); 

		if(error_setting[0] || error_setting[1] != ESP_OK){
			ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",0,
			error_setting[0] != ESP_OK ? "SENSOR 0x68" :
			error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
		}
		error_setting[0]  = i2c_imu_setup(1,SLAVE1_ADD); 
		//error_setting[1]  = i2c_imu_setup(1,SLAVE2_ADD); 

		if(error_setting[0] || error_setting[1] != ESP_OK){
			ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",1,
			error_setting[0] != ESP_OK ? "SENSOR 0x68" :
			error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
		} 	
  	addr ++;

	}
	/*
	error_setting[0]  = i2c_imu_setup(0,SLAVE1_ADD); 
	error_setting[1]  = i2c_imu_setup(0,SLAVE2_ADD); 

	if(error_setting[0] || error_setting[1] != ESP_OK){
		ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",0,
		error_setting[0] != ESP_OK ? "SENSOR 0x68" :
		error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
	}
	
	gpio_set_level(pinA, 1);
	gpio_set_level(pinB, 0);
	error_setting[0]  = i2c_imu_setup(0,SLAVE1_ADD); 
	error_setting[1]  = i2c_imu_setup(0,SLAVE2_ADD); 

	if(error_setting[0] || error_setting[1] != ESP_OK){
		ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",master_num,
		error_setting[0] != ESP_OK ? "SENSOR 0x68" :
		error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
	}
	*/
}

void buffer_arrange(Glove* glove, char message[]){

	sprintf(message,"%.2f|%.2f|%.2f|%.2f|%d|%.2f|%.2f|%.2f|%.2f|%d|%.2f|%.2f|%.2f|%.2f|%d|%.2f|%.2f|%.2f|%.2f|%d|%.2f"
		"|%.2f|%.2f|%.2f|%d\n",
	glove->fingers[0].medial.theta*degre_conv,
	glove->fingers[0].medial.phi*degre_conv,
	glove->fingers[0].proximal.theta*degre_conv,
	glove->fingers[0].proximal.phi*degre_conv,
	glove->fingers[0].pressure,
	glove->fingers[1].medial.theta*degre_conv,
	glove->fingers[1].medial.phi*degre_conv,
	glove->fingers[1].proximal.theta*degre_conv,
	glove->fingers[1].proximal.phi*degre_conv,
	glove->fingers[1].pressure,
	glove->fingers[2].medial.theta*degre_conv,
	glove->fingers[2].medial.phi*degre_conv,
	glove->fingers[2].proximal.theta*degre_conv,
	glove->fingers[2].proximal.phi*degre_conv,
	glove->fingers[2].pressure,
	glove->fingers[3].medial.theta*degre_conv,
	glove->fingers[3].medial.phi*degre_conv,
	glove->fingers[3].proximal.theta*degre_conv,
	glove->fingers[3].proximal.phi*degre_conv,
	glove->fingers[3].pressure,
	glove->fingers[4].proximal.theta*degre_conv,
	glove->fingers[4].proximal.phi*degre_conv,
	glove->frame_reference.theta*degre_conv,
	glove->frame_reference.phi*degre_conv,
	glove->fingers[4].pressure);
}

void raw_data_zero(raw_data* member){
		member->accelx=0;
		member->accely=0;
		member->accelz=0;
		member->gyrox=0;
		member->gyroy=0;
		member->gyroz=0;
}

void buffer_raw_data(raw_data* member1,raw_data* member2){
	char message[256];
	sprintf(message,"%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d\n",
		member1->master_num,
		member1->finger,
		member1->accelx,
		member1->accely,
		member1->accelz,
		member1->gyrox,
		member1->gyroy,
		member1->gyroz,
		member2->master_num,
		member2->finger,
		member2->accelx,
		member2->accely,
		member2->accelz,
		member2->gyrox,
		member2->gyroy,
		member2->gyroz);
		ESP_LOGI(TAG,"%s",message);
}
