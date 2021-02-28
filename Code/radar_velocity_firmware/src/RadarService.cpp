#include "RadarService.h"

RadarService::RadarService(	float pt,
							uint32_t sr,
							uint8_t  dr,
							uint16_t sl,
							uint8_t  sp
){
									
	peak_threshold = pt;
	sweep_rate = sr;
	distance_res = dr;
	sweep_length = sl;
	sweeps = sp;
	distance_bins = sl/dr;
}

uint8_t RadarService::get_byte(uint32_t val, uint8_t byte){
	return (val & (0xFF << (8*byte))) >> (8*byte);
}

uint32_t RadarService::read_reg(uint8_t reg){
	//0xCC, 0x01, 0x00, 0xF8, reg, 0xCD
	clr_cmd();
	
	cmd_str[0] = 0xCC;
	cmd_str[1] = 0x01;
	cmd_str[2] = 0x00;
	cmd_str[3] = 0xF8;
	cmd_str[4] = reg;
	cmd_str[5] = 0xCD;

	send(6);
	
	int16_t j = 0;
	
	int32_t value = 0;
	int16_t bt;
	
	do{
		bt = read_next();
		
		if (bt == -2){
			j = -1;
			continue;
		}
		
		
		if (0 <= bt and bt <= 255){
		value |= read_next() << (j%4)*8;
		}
		
		
		j++;
	}while(bt != -1);
	return value;
}

void RadarService::read_buf(int16_t *data){
	//0XCC, 0x03, 0x00, 0xFA, 0xE8, 0x00, 0x00, 0xCD
	clr_cmd();
	
	cmd_str[0] = 0xCC;
	cmd_str[1] = 0x03;
	cmd_str[2] = 0x00;
	cmd_str[3] = 0xFA;
	cmd_str[4] = 0xE8;
	cmd_str[5] = 0x00;
	cmd_str[6] = 0x00;
	cmd_str[7] = 0xCD;
	
	send(8);
	
	int16_t j = 0;
	
	int32_t value;
	int16_t bt = read_next();
	do{
		
		if (bt == -2){
			j = -1;
			continue;
		}
		
		if (j%2 == 0){
			value = 0;
		}
		
		if (0 <= bt and bt <= 255){
		value |= read_next() << (j%2)*8;
		}
		
		if (j%2 == 1){
			value -= 0x8000;
			data[j/2] = value;
		}
		
		j++;
	}while(bt != -1);
	
}

void RadarService::measure_sparce(int32_t sd){
	start_distance = sd;
	//close sensor
	write_reg(0x03,0x00);
	//clear status
	write_reg(0x03,0x04);
	read_reg(0x06);
	//sparse mode
	write_reg(0x02,0x04);
	//close sensor
	write_reg(0x03,0x00);
	read_reg(0x06);
	//sparse mode
	write_reg(0x02,0x04);

	//set no software upper limit on rate we can poll sensor
	write_reg(0x23,0);
	//set power mode to active
	write_reg(0x25,0x03);
	//set profile to 3
	write_reg(0x28,0x03);
	//set sparse sampling mode to A
	write_reg(0x42,0x00);
	//set repition mode to be limited to update rate
	write_reg(0x22,0x02);
	//set sweep rate to 1 000 000 mHz
	write_reg(0x41, sweep_rate);


	//set start distace to 240 mm
	write_reg(0x20,start_distance);
	//set length to 560 mm
	write_reg(0x21,sweep_length);

	//set gain to 500
	write_reg(0x24,500);
	//set downsampling factor to 1
	write_reg(0x29,1);
	//set HW_ACC_AVE_SAMPLES to 32
	write_reg(0x30,32);
	//set sweeps per frame to 64
	write_reg(0x40, sweeps);
	//disable attenuation
	write_reg(0x32, 0x00);
	//enable TX
	write_reg(0x26, 0x00);


	//enable asynchonus measurement
	write_reg(0x33, 0x01);
	//set power mode to active
	write_reg(0x25, 0x03);
	//enable uart streaming
	write_reg(0x05, 0x00);
	//create service
	write_reg(0x03, 0x01);

	///////////////////////ADD CHECK CODE HERE///////////////////
	// //check state
	// read_reg(0x06);

	// //read start
	// read_reg(0x81);
	// //read len
	// read_reg(0x82);
	// //read data length
	// read_reg(0x83);
	// //read sweep rate
	// read_reg(0x84);
	// //read step length
	// read_reg(0x85);

	//active service
	write_reg(0x03,0x02);

	//block untill data taken
	while(not (read_reg(0x06) & 0x00000100));


}


void RadarService::send(uint8_t len){
	Serial.write(cmd_str, len);
}

void RadarService::clr_cmd(){
	memset(cmd_str, '\0', 10);
}


uint8_t RadarService::read(){
	while (not Serial.available()){}
	return Serial.read();
}

int16_t RadarService::read_next(){

	
	if(i == end){
		end = -1;
		i = 0;
		length = 0;
		return -1;
	}
	
	uint8_t bt = read();
	//set the length of data
	if (i == 1){length =  bt;}
	if (i == 2){length |=  bt << 8; end = length + 5;}
	
	if(4 < i and i < end){
		i++;
		return bt;	
	}else{
		i++;
		return -2;
	}
	
}

void RadarService::write_reg(uint8_t reg, uint32_t val){
	
	//0xCC, 0x05, 0x00, 0xF9, reg, get_byte(val,0), get_byte(val,1), get_byte(val,2), get_byte(val,3), 0xCD
	
	clr_cmd();
	
	cmd_str[0] = 0xCC;
	cmd_str[1] = 0x05;
	cmd_str[2] = 0x00;
	cmd_str[3] = 0xF9;
	cmd_str[4] = reg;
	cmd_str[5] = get_byte(val,0);
	cmd_str[6] = get_byte(val,1);
	cmd_str[7] = get_byte(val,2);
	cmd_str[8] = get_byte(val,3);
	cmd_str[9] = 0xCD;
	
	send(10);
}