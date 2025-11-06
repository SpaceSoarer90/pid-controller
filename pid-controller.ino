#include "PinChangeInterrupt.h"
#include <TimerOne.h>

#define MOTOR_PIN_MINUS    7
#define MOTOR_PIN_PLUS     8
#define MOTOR_PIN_PWM      3
#define MOTOR_PIN_ENC      12

#define SAMPLING_INDICATOR_LED  4

typedef struct {
  float kp, ki, kd;
  
  volatile bool sample_ready;
  uint32_t sampling_interval;
  uint32_t time_last;
  
  float setpoint, error;
  volatile float currpoint;
  float currpoint_last;
  float proportional, integral, derivative;

  float raw_output, output, max_output;

  struct {
    uint8_t pwm_output;
    bool saturated;
    float currpoint_snapshot;
    float p_term, i_term, d_term;
    float calculation_time;
    float dt;
  } telemetry;
} pid_system_t;

typedef struct {
  struct {
    volatile uint16_t curr_pulses;
    volatile uint16_t last_pulses;
    volatile uint16_t delta_pulses;
  } enc;
  
} motor_system_t;

enum PID_CONSTANTS {
  PID_KP = 0,
  PID_KI,
  PID_KD
};

pid_system_t pid = { 
  .kp = 0.40, .ki = 0.02, .kd = 0.001,

  .sample_ready = false,
  .sampling_interval = 100,
  .time_last = 0,

  .setpoint = 0, .error = 0,
  .currpoint = 0,
  .currpoint_last = 0,
  .proportional = 0, .integral = 0, .derivative = 0,

  .raw_output = 0, .output = 0,
};

motor_system_t motor = {
  .enc = { 0, 0, 0 }
};

float* pid_constant_ptrs[] = { &pid.kp, &pid.ki, &pid.kd };
byte pid_constant_idx = PID_KP;

void setup() {
  pinMode(MOTOR_PIN_MINUS, OUTPUT);
  pinMode(MOTOR_PIN_PLUS, OUTPUT);

  pinMode(SAMPLING_INDICATOR_LED, OUTPUT);

  analogWrite(MOTOR_PIN_PWM, 0);
  digitalWrite(MOTOR_PIN_MINUS, HIGH);
  digitalWrite(MOTOR_PIN_PLUS, LOW);
  
  attachPCINT(digitalPinToPCINT(MOTOR_PIN_ENC), enc_pulse_ISR, CHANGE); // have double the resolution for free!
  // attachPCINT(digitalPinToPCINT(MOTOR_PIN_ENC), enc_pulse_ISR, FALLING);
  
  Timer1.initialize(1000 * (unsigned long)pid.sampling_interval);
  Timer1.attachInterrupt(sample_enc_ISR);
  
  Serial.begin(115200);
  // Serial.println("Hello world!");
  // Serial.println("# you should've seen something like \"something's wrong\"");
  
  digitalWrite(SAMPLING_INDICATOR_LED, HIGH);
  delay(1000);
  digitalWrite(SAMPLING_INDICATOR_LED, LOW);
  delay(500);

  Serial.println("Sampling MAX_MOTOR_RPM, please standby...");
  pid.max_output = sample_max_rpm_process(10 * 1000);
}

void loop() {
  input_process();
  
  if (pid.sample_ready) {
    pid.sample_ready = false;

    uint32_t calculation_started = millis();
    float pid_output = pid_compute_process(&pid);
    motor_output_process(pid_output);
    pid.telemetry.calculation_time = float(millis() - calculation_started) / 1000.0f;

    print_data();
  }
  delay(10);
}

float sample_max_rpm_process(uint32_t sampling_duration) {
  analogWrite(MOTOR_PIN_PWM, 255);
  digitalWrite(MOTOR_PIN_MINUS, HIGH);
  digitalWrite(MOTOR_PIN_PLUS, LOW);
  
  delay(2500); // wait for motor to reach max speed

  float rpm_sum = 0;
  float currpoint_snapshot = 0;
  uint16_t sample_count = 0;
  uint32_t start_time = millis();

  while ((millis() - start_time) < sampling_duration) {
    if (pid.sample_ready) {
      noInterrupts();
      currpoint_snapshot = pid.currpoint;
      interrupts();

      rpm_sum += currpoint_snapshot;
      sample_count++;
      pid.sample_ready = false;
      Serial.print("Sample Received (rpm_sum, sample_count): ");
      Serial.print(rpm_sum, 2);
      Serial.print(", ");
      Serial.println(sample_count);
    }
  }

  analogWrite(MOTOR_PIN_PWM, 0);

  Serial.println("Finished sampling MOTOR MAX RPM!");
  Serial.print("MOTOR_MAX_RPM value: ");
  Serial.println(rpm_sum / float(sample_count), 2);


  delay(2500);

  return rpm_sum / float(sample_count);
}

void enc_pulse_ISR() {
  motor.enc.curr_pulses++;

  digitalWrite(SAMPLING_INDICATOR_LED, !digitalRead(SAMPLING_INDICATOR_LED));
}

void sample_enc_ISR() {
  const float DELTA_T = pid.sampling_interval / 1000.0f;
  const float EMA_CONSTANT = 0.35;
  // const float TICKS_PER_REV = 40.0f; // it counts both the falling and rising edges
  const float TICKS_PER_REV = 20.0f;

  motor.enc.delta_pulses = motor.enc.curr_pulses - motor.enc.last_pulses;
  motor.enc.last_pulses = motor.enc.curr_pulses;
  
  float revs_per_sec = (((float)motor.enc.delta_pulses / TICKS_PER_REV) / DELTA_T);
  float revs_per_min = revs_per_sec * 60.0f;  

  // Apply exponential moving average filter to reduce jitter
  pid.currpoint = EMA_CONSTANT * revs_per_min + (1 - EMA_CONSTANT) * pid.currpoint;

  // digitalWrite(SAMPLING_INDICATOR_LED, !digitalRead(SAMPLING_INDICATOR_LED));

  pid.sample_ready = true;
}

int sgn(float val) {
  return (val > 0) - (val < 0);
}

float pid_compute_process(pid_system_t* p) {
  float dt;
  uint32_t time_now;
  float currpoint_snapshot;
  bool output_saturated, output_accelerating;
  
  p->raw_output = p->kp * p->error +
              p->ki * p->integral +
              p->kd * p->derivative;
  
  p->output = constrain(p->raw_output, 0.0f, p->max_output);
  
  noInterrupts();
  currpoint_snapshot = p->currpoint;
  p->telemetry.currpoint_snapshot = p->currpoint;
  interrupts();
  
  time_now = millis();
  dt = float(time_now - p->time_last) / 1000.0f;
  
  p->error = p->setpoint - currpoint_snapshot;
  
  output_saturated = (p->raw_output != p->output);
  output_accelerating = (sgn(p->error) == sgn(p->output));
  
  if (!output_saturated || !output_accelerating) {
    p->integral += p->error * dt;
  }
  
  p->derivative = -(currpoint_snapshot - p->currpoint_last) / dt;
  
  p->proportional = p->error;
  
  p->time_last = time_now;
  p->currpoint_last = currpoint_snapshot;


  // set telemetry data
  p->telemetry.p_term = p->kp * p->proportional;
  p->telemetry.i_term = p->ki * p->integral;
  p->telemetry.d_term = p->kd * p->derivative;
  p->telemetry.dt = dt;
  // p->telemetry.currpoint_snapshot = currpoint_snapshot;
  
  return p->output;
}

void motor_output_process(float input_rpm) {
  uint8_t output_pwm = abs((input_rpm / pid.max_output) * 255);

  pid.telemetry.pwm_output = output_pwm;

  analogWrite(MOTOR_PIN_PWM, output_pwm);
}

void input_process() {
  while (Serial.available()) {
    char char_in = Serial.read();
    
    switch (char_in) {
      case 'u':
        pid.sampling_interval -= 10;
        break;
        
      case 'i':
        pid.sampling_interval += 10;
        Serial.print("Changed Sampling interval: ");
        Serial.println(pid.sampling_interval);
        pid.max_output = sample_max_rpm_process(10 * 1000);
        break;
        
      // yeah i fucking hate this too
      case 'r':
        // clear the buffer for extraneous \r\n
        // while (Serial.available()) {
        //   Serial.println("CLEARING!!");
        //   Serial.println(Serial.read());
        // }

        Serial.print("Set Target RPM: ");

        bool rpm_set = false;
        String recv_command;
        while (!rpm_set) {
          if (Serial.available()) {
            char recv = Serial.read();

            recv_command += recv;
            Serial.print(recv);
            Serial.flush();

            bool is_recv_digit = (recv >= '0' && recv <= '9');
            bool command_done = (recv == '!');

            if ((!is_recv_digit && !command_done)) {
              Serial.print("this da character: (");
              Serial.print(recv);
              Serial.println(")");
              Serial.println("invalid command");
              break;
            }

            if (command_done && (recv_command.length() <= 1)) {
              Serial.print("\r\nno rpm specified, setting to default value 0.");
              recv_command = "0";
            }

            if (command_done) {
              Serial.println();
              // Serial.println("Command set, changing target RPM now...");

              recv_command.remove(recv_command.indexOf('!'));
              pid.setpoint = recv_command.toFloat();
          
              Serial.print("New Target RPM: ");
              Serial.println(pid.setpoint);

              break;
            }
          }
        }
        break;
    }
    
    Timer1.setPeriod((unsigned long)pid.sampling_interval * 1000);
  }
}

void print_data() {
  Serial.print("# ");
  Serial.print(float(millis()) / 1000.0f, 3);
  Serial.print(',');

  Serial.print(pid.setpoint, 5);
  Serial.print(',');
  Serial.print(pid.currpoint, 5);
  Serial.print(',');
  Serial.print(pid.error, 5);
  Serial.print(',');
  Serial.print(pid.output, 5);
  Serial.print(',');

  Serial.print(pid.raw_output, 5);
  Serial.print(',');
  Serial.print(pid.telemetry.p_term, 5);
  Serial.print(',');
  Serial.print(pid.telemetry.i_term, 5);
  Serial.print(',');
  Serial.print(pid.telemetry.d_term, 5);
  Serial.print(',');
  Serial.print(pid.kp, 5);
  Serial.print(',');
  Serial.print(pid.ki, 5);
  Serial.print(',');
  Serial.print(pid.kd, 5);
  Serial.print(',');
  Serial.print(pid.telemetry.dt, 3);
  Serial.print(',');

  Serial.print(pid.telemetry.pwm_output, 5);
  Serial.print(',');
  Serial.print(float(pid.sampling_interval) / 1000.0f, 1); // in seconds
  Serial.print(',');
  Serial.print(pid.telemetry.calculation_time, 5);
  Serial.print(',');
  Serial.print(pid.telemetry.saturated, 1);

  // some other data
  Serial.print(motor.enc.delta_pulses);
  Serial.println();
}
