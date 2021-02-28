#include <stdint.h>
#include <Arduino.h>
 
class RadarService{
	public:
		RadarService(float pt, uint32_t sr, uint8_t dr, uint16_t sl, uint8_t sp);
		int16_t read_next(); 
		uint32_t read_reg();
		void read_buf(int16_t *data);
		void write_reg(uint8_t, uint32_t);
		uint32_t read_reg(uint8_t);
		void measure_sparce(int32_t);
	
		
		float peak_threshold;
		uint32_t sweep_rate;
		uint8_t  distance_res;
		uint16_t sweep_length;
		uint8_t  sweeps;
		uint8_t  distance_bins;
		int32_t start_distance;

	private:
	
		int16_t end = -1;
		int16_t i = 0;
		int16_t length;
		uint8_t read();
		uint8_t cmd_str[10];
		void send(uint8_t);
		void clr_cmd();
		uint8_t get_byte(uint32_t, uint8_t);
};