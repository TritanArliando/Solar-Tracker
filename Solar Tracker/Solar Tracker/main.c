/*
 * Solar Tracker.c
 *
 * Created: 03/07/2024 20.27.41
 * Author : Tritan
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Deklarasi pin LDR
#define LDR_TL 0 // ADC0
#define LDR_TR 1 // ADC1
#define LDR_BL 2 // ADC2
#define LDR_BR 3 // ADC3

// Pin servo
#define SERVO1_PIN PB1 // OC1A
#define SERVO2_PIN PB2 // OC1B

// Variabel untuk posisi servo
volatile int pos1 = 90; // Posisi awal servo 1
volatile int pos2 = 90; // Posisi awal servo 2
int tol = 10; // Toleransi untuk perbedaan nilai

// Variabel PID
float Kp1 = 1.0, Ki1 = 0.01, Kd1 = 0.5; // servo 1
float Kp2 = 1.0, Ki2 = 0.01, Kd2 = 0.5; // servo 2

// Error sebelumnya dan integral error untuk servo 1 dan 2
float prevError1 = 0, integral1 = 0;
float prevError2 = 0, integral2 = 0;

// Waktu sebelumnya untuk menghitung delta waktu
volatile unsigned long prevTime1 = 0, prevTime2 = 0;

void adc_init() {
	// Inisialisasi ADC
	ADMUX = (1 << REFS0); // AVcc dengan eksternal kapasitor di AREF
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler 64
}

uint16_t adc_read(uint8_t ch) {
	// Pilih saluran ADC
	ADMUX &= 0xF0;
	ADMUX |= ch;

	// Memulai konversi
	ADCSRA |= (1 << ADSC);

	// Tunggu konversi selesai
	while (ADCSRA & (1 << ADSC));

	return ADC;
}

void servo_init() {
	// Inisialisasi Timer1 untuk PWM
	TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Mode 14, prescaler 8
	ICR1 = 19999; // TOP value untuk 50Hz (20ms periode)
	DDRB |= (1 << SERVO1_PIN) | (1 << SERVO2_PIN);
}

void set_servo1_position(int angle) {
	OCR1A = (angle * 11) + 1000; // Konversi derajat ke nilai OCR
}

void set_servo2_position(int angle) {
	OCR1B = (angle * 11) + 1000; // Konversi derajat ke nilai OCR
}

unsigned long get_millis() {
	// Menggunakan Timer0 untuk melacak waktu
	return (unsigned long)(TCNT0 * 64.0 / (F_CPU / 1000.0));
}

int main(void) {
	adc_init();
	servo_init();
	sei(); // Enable global interrupt

	set_servo1_position(pos1);
	set_servo2_position(pos2);

	prevTime1 = get_millis();
	prevTime2 = get_millis();

	while (1) {
		// Membaca nilai dari setiap LDR
		int topLeft = adc_read(LDR_TL);
		int topRight = adc_read(LDR_TR);
		int bottomLeft = adc_read(LDR_BL);
		int bottomRight = adc_read(LDR_BR);

		// Menghitung rata-rata untuk bagian atas dan bawah
		int avgTop = (topLeft + topRight) / 2;
		int avgBottom = (bottomLeft + bottomRight) / 2;
		
		// Menghitung rata-rata untuk bagian kiri dan kanan
		int avgLeft = (topLeft + bottomLeft) / 2;
		int avgRight = (topRight + bottomRight) / 2;

		// Menghitung error untuk servo 1 dan 2
		float error1 = avgRight - avgLeft;
		float error2 = avgBottom - avgTop;

		// Menghitung delta waktu untuk servo 1
		unsigned long currentTime1 = get_millis();
		float deltaTime1 = (currentTime1 - prevTime1) / 1000.0;

		// Menghitung integral dari error untuk servo 1
		integral1 += error1 * deltaTime1;

		// Menghitung derivative dari error untuk servo 1
		float derivative1 = (error1 - prevError1) / deltaTime1;

		// Menghitung output PID untuk servo 1
		float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;

		// Menggerakkan servo 1
		if (abs(avgLeft - avgRight) > tol) {
			if (avgLeft > avgRight) {
				pos1 = pos1 - 1;
				} else {
				pos1 = pos1 + 1;
			}
			pos1 = (pos1 < 0) ? 0 : (pos1 > 180) ? 180 : pos1;
			set_servo1_position(pos1);
		}
		
		// Simpan error dan waktu sebelumnya untuk servo 1
		prevError1 = error1;
		prevTime1 = currentTime1;

		// Delay untuk menghindari perubahan posisi yang terlalu cepat
		_delay_ms(50);

		// Menghitung delta waktu untuk servo 2
		unsigned long currentTime2 = get_millis();
		float deltaTime2 = (currentTime2 - prevTime2) / 1000.0;

		// Menghitung integral dari error untuk servo 2
		integral2 += error2 * deltaTime2;

		// Menghitung derivative dari error untuk servo
		float derivative2 = (error2 - prevError2) / deltaTime2;

		// Menghitung output PID untuk servo 2
		float output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;

		// Menggerakkan servo 2
		if (abs(avgTop - avgBottom) > tol) {
			if (avgTop > avgBottom) {
				pos2 = pos2 - 1;
				} else {
				pos2 = pos2 + 1;
			}
			pos2 = (pos2 < 0) ? 0 : (pos2 > 180) ? 180 : pos2;
			set_servo2_position(pos2);
		}

		// Simpan error dan waktu sebelumnya untuk servo 2
		prevError2 = error2;
		prevTime2 = currentTime2;

		// Delay untuk menghindari perubahan posisi yang terlalu cepat
		_delay_ms(100); // Menambahkan delay setelah setiap iterasi loop
	}
}

