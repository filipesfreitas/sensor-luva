#include <position_process.h>

const static float alpha = time_constant/(time_constant+period);

void orientation_estimation(raw_data metacarpo,raw_data proximal,Glove* glove,int i) {

	/* PHI	*/
	if (proximal.accelx + proximal.accely + proximal.accelz == 0) {

	}
	else {
		glove->fingers[i].proximal.phi =
		atan(sqrt(pow(proximal.accelx,2) + pow(proximal.accely,2) )/proximal.accelz)*degre_conv*(1-alpha) +
		(glove->fingers[i].proximal.phi + proximal.gyroy * period ) * alpha ;
	}

	if (metacarpo.accelx + metacarpo.accely + metacarpo.accelz == 0){

	}
	else {
		glove->fingers[i].metacarpo.phi =
		atan(sqrt(pow(metacarpo.accelx,2) + pow(metacarpo.accely,2))/	metacarpo.accelz)*degre_conv*(1-alpha) +
		(glove->fingers[i].metacarpo.phi + metacarpo.gyroy * period ) * alpha;
	}
	/* THETA */
	if (metacarpo.accelx==0) glove->fingers[i].metacarpo.theta = 0;
	else glove->fingers[i].metacarpo.theta = atan(metacarpo.accely/metacarpo.accelx)*degre_conv*(1-alpha) + 
		alpha * (glove->fingers[i].metacarpo.theta + metacarpo.gyroz * period);

	if (proximal.accelx==0)	glove->fingers[i].proximal.theta = 0;
	else glove->fingers[i].proximal.theta = atan(proximal.accely/proximal.accelx)*degre_conv*(1-alpha) +
		alpha * (glove->fingers[i].proximal.theta + proximal.gyroz * period);

	glove->fingers[i].metacarpo.phi = glove->fingers[i].metacarpo.phi - glove -> frame_reference.phi;
	glove->fingers[i].proximal.phi  = glove->fingers[i].proximal.phi - glove->fingers[i].metacarpo.phi
	- glove -> frame_reference.phi;
	glove->fingers[i].metacarpo.theta = glove -> fingers[i].metacarpo.theta - glove -> frame_reference.theta ;
	glove->fingers[i].proximal.theta  = glove -> fingers[i].proximal.theta - glove -> fingers[i].metacarpo.theta 
	- glove -> frame_reference.theta;
}
void buffer_raw_data(raw_data* member1,raw_data* member2){
	char message[256];
	static float gyro_angle_x = 0;
	static float gyro_angle_y = 0;
	static float gyro_angle_z = 0;

	float theta = atan(member1->accelx/member1->accely)*degre_conv;
	float phi = acos(member1->accelz/sqrt(pow(member1->accelx,2) + pow(member1->accely,2) +pow(member1->accelz,2)))*degre_conv;
	float accel_angle_z = 0;
	
	gyro_angle_x += member1->gyrox*50e-3;
	gyro_angle_y += member1->gyroy*50e-3;
	gyro_angle_z += member1->gyroz*50e-3;

	float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*phi;
	float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*theta;
	float angle_z = gyro_angle_z;  
	
	sprintf(message,"acc_angle:%f\tgyro_angle:%f\tangle_comp:%f\n",	phi,gyro_angle_y, angle_x);
	gyro_angle_x=angle_x;
	printf("%s",message);
}

void reference_frame_orientation(raw_data reference,Glove* glove){
	/* PHI	*/
	glove->frame_reference.phi = acos(reference.accelz/(sqrt(reference.accelx*reference.accelx)+
		reference.accely*reference.accely+reference.accelz*reference.accelz))*(1-alpha)*degre_conv + 
	(glove->frame_reference.phi + reference.gyroy*gyrosens * period ) * alpha;
	/* THETA */
	if (reference.accely==0) glove->frame_reference.theta = 0;
	else glove->frame_reference.theta=reference.gyroz * period;//glove->frame_reference.theta = glove->frame_reference.theta*degre_conv + reference.gyroz * period;

}

void calibration(Glove* glove){
	int addr = 0;
	int ret, ret1;
	int16_t buffer[7];
	uint8_t sensor[14];
	uint8_t sensor2[14];

	memset(sensor,0,14);
	memset(sensor2,0,14);
	memset(buffer,0,14);

	/* Mux CONFIG*/
	mux_selector_config();


	while (addr < channels ) {
		gpio_set_level(pinA, (addr & 2) >> 1);
		gpio_set_level(pinB, addr & 1);
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
			buffer[6] = adc_read(addr,adc_chars);
			buffer[6] = buffer[6]/(Vinput - buffer[6])*R;


			/* PHI	*/
			glove->fingers[addr].proximal.phi = acos(buffer[2]/(sqrt(pow(buffer[0],2)+pow(buffer[1],2)+pow(buffer[2],2))));
			glove->fingers[addr].metacarpo.phi = acos(buffer[5]/(sqrt(pow(buffer[3],2)+pow(buffer[4],2)+pow(buffer[5],2))));

			/* THETA */
			if (buffer[1]==0) glove->fingers[addr].proximal.theta = 0;
			else glove->fingers[addr].proximal.theta = atan(buffer[0]/buffer[1]);

			if (buffer[4]==0) glove->fingers[addr].metacarpo.theta = 0;
			else glove->fingers[addr].metacarpo.theta = atan(buffer[3]/buffer[4]);
		}
		addr++;
	}
}

void initialization(Glove* glove){

	glove->fingers[0].metacarpo.theta=0;
	glove->fingers[0].metacarpo.phi=0;
	glove->fingers[0].proximal.theta=0;
	glove->fingers[0].proximal.phi=0;
	glove->fingers[0].pressure=0;
	glove->fingers[1].metacarpo.theta=0;
	glove->fingers[1].metacarpo.phi=0;
	glove->fingers[1].proximal.theta=0;
	glove->fingers[1].proximal.phi=0;
	glove->fingers[1].pressure=0;
	glove->fingers[2].metacarpo.theta=0;
	glove->fingers[2].metacarpo.phi=0;
	glove->fingers[2].proximal.theta=0;
	glove->fingers[2].proximal.phi=0;
	glove->fingers[2].pressure=0;
	glove->fingers[3].metacarpo.theta=0;
	glove->fingers[3].metacarpo.phi=0;
	glove->fingers[3].proximal.theta=0;
	glove->fingers[3].proximal.phi=0;
	glove->fingers[3].pressure=0;
	glove->fingers[4].proximal.theta=0;
	glove->fingers[4].proximal.phi=0;
	glove->frame_reference.theta=0;
	glove->frame_reference.phi=0;
	glove->fingers[4].pressure=0;



}
void setup_sensors(){
	uint8_t addr=0; 
	/* aux variable for counting*/
	esp_err_t error_setting[2]={ESP_OK,ESP_OK}; 

	/* Mux CONFIG*/
	mux_selector_config();
	/**
	 * first set reference sensor	 
	 */
	gpio_set_level(pinA, 0);
	gpio_set_level(pinB, 1);
	error_setting[0]  = i2c_imu_setup_reference(1,SLAVE1_ADD); 

	if(error_setting[0] || error_setting[1] != ESP_OK){
		ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",0,
			error_setting[0] != ESP_OK ? "SENSOR 0x68" :
			error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
	}		
	while(addr < channels){
		gpio_set_level(pinA, (addr & 2) >> 1);
		gpio_set_level(pinB, addr & 1);
		error_setting[0]  = i2c_imu_setup(0,SLAVE1_ADD); 
		error_setting[1]  = i2c_imu_setup(0,SLAVE2_ADD); 

		if(error_setting[0] || error_setting[1] != ESP_OK){
			ESP_LOGE(TAG, "Problem at master nº: %d\tNo ack, sensor %s not connected...skip...\n",0,
				error_setting[0] != ESP_OK ? "SENSOR 0x68" :
				error_setting[1] != ESP_OK ? "SENSOR 0x69" : "None");
		}		
		addr ++;

	}

}

void buffer_arrange(Glove* glove, char message[]){

	sprintf(message,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
		glove->fingers[0].metacarpo.theta,
		glove->fingers[0].metacarpo.phi,
		glove->fingers[0].proximal.theta,
		glove->fingers[0].proximal.phi,
		glove->fingers[0].pressure,
		glove->fingers[1].metacarpo.theta,
		glove->fingers[1].metacarpo.phi,
		glove->fingers[1].proximal.theta,
		glove->fingers[1].proximal.phi,
		glove->fingers[1].pressure,
		glove->fingers[2].metacarpo.theta,
		glove->fingers[2].metacarpo.phi,
		glove->fingers[2].proximal.theta,
		glove->fingers[2].proximal.phi,
		glove->fingers[2].pressure,
		glove->fingers[3].metacarpo.theta,
		glove->fingers[3].metacarpo.phi,
		glove->fingers[3].proximal.theta,
		glove->fingers[3].proximal.phi,
		glove->fingers[3].pressure,
		glove->fingers[4].proximal.theta,
		glove->fingers[4].proximal.phi,
		glove->frame_reference.theta,
		glove->frame_reference.phi,
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

