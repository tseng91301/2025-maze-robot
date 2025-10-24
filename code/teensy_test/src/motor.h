#ifndef MOTOR_H
#define MOTOR_H 1
#include <Arduino.h>
class Motor {
    private:
        int pwm_p;
        int in1_p;
        int in2_p;

        // Encoder
        volatile long pulseCount = 0;
        int enc_A;
        int enc_B;

        // Speed reading
        unsigned long read_speed_interval = 100; // ms
        unsigned long last_read_speed_time = 0; // ms
        float speed_now = 0.0;
        float target_speed = 0.0;

        void update_speed() {
            long count;
            noInterrupts();
            count = this->pulseCount;
            this->pulseCount = 0;
            interrupts();

            const float PPR_out = 11 * 30; // 輸出軸每圈脈衝數
            this->speed_now = (count / PPR_out) * (60.0 / 0.1);
        }

        void output(int val) {
            /* 直接對馬達腳位輸出 pwm 訊號 */
            // Clipping
            if(val > 255) val = 255;
            if (val < -255) val = -255;

            // 設定方向
            if (val >= 0) {
                digitalWrite(this->in1_p, HIGH);
                digitalWrite(this->in2_p, LOW);
            } else {
                digitalWrite(this->in1_p, LOW);
                digitalWrite(this->in2_p, HIGH);
            }

            analogWrite(this->pwm_p, abs(val));
        }

    public:
        // 設定是否使用 encoder
        bool use_encoder = true;

        Motor(int pwm, int in1, int in2, int encA, int encB) {
            this->pwm_p = pwm;
            this->in1_p = in1;
            this->in2_p = in2;
            this->enc_A = encA;
            this->enc_B = encB;
            pinMode(this->pwm_p, OUTPUT);
            pinMode(this->in1_p, OUTPUT);
            pinMode(this->in2_p, OUTPUT);
            pinMode(this->enc_A, INPUT_PULLUP);
            pinMode(this->enc_B, INPUT_PULLUP);

            // 設定 PWM 頻率與解析度
            analogWriteFrequency(this->pwm_p, 20000); // 20kHz PWM
            analogWriteResolution(8); // 0~max_pwm
        }

        void handle_encoder() {
            int b = digitalRead(this->enc_B);
            if (b == HIGH) this->pulseCount += 1;
            else this->pulseCount -= 1;
        }

        void set_speed(float spd) {
            // 輸入 RPM 數值, 作為速度的目標
            if (!(this->use_encoder)) {
                output((int)spd);
                this->speed_now = spd;
                return;
            }
            this->target_speed = spd;
            return;
        }

        float speed() {
            return this->speed_now;
        }

        void service() {
            unsigned long time_now = millis();
            if ((time_now - this->last_read_speed_time > this->read_speed_interval) && (this->use_encoder)) {
                this->last_read_speed_time = time_now;
                update_speed();
            }
        }
};
#endif