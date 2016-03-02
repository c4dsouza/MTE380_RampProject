void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


// PID library
class PID
{
  public:
    PID();
    void tune(double p, double i, double d);
    void setTimeslice(int t);
    void getInputFunction(double (*a)());
    void getOutputFunction(double (*a)());
    void onControlSignal(void (*a)(double c));
    void delayRefresh();
    int refresh();
  private:
    double kp;
    double kd;
    double ki;
    double dt; // IN MICROS
    double prevError;
    double errorSum;
    int timestamp;
    unsigned int timeDifference(int now, int then);
    double (*input)();
    double (*output)();
    void (*control)(double c);
};

PID::PID()
{
  kp = 1;
  kd = 0;
  ki = 0;
  dt = 1000;
  timestamp = micros();
  prevError = 0;
  errorSum = 0;
}

void PID::tune(double p, double i, double d) {
  kp = p;
  kd = d;
  ki = i;
}

void PID::setTimeslice(int t) {
  dt = t;
}

int PID::refresh() {
  unsigned int now = micros();
  unsigned int delta = timeDifference(now, timestamp);
  if(delta < dt) {
    return 0;
  }

  double error = input() - output();
  errorSum += ki*error*delta;
  double c = kp*error + kd*(error-prevError)/delta + errorSum;
  prevError = error;

  control(c);

  timestamp = now;

  return 1;
}

void PID::delayRefresh() {
  unsigned int delta = timeDifference(micros(), timestamp);
  delayMicroseconds(delta);
  while(!refresh()){};
}

unsigned int PID::timeDifference(int now, int then) {
  return ((unsigned int)now-then);
}

void PID::getInputFunction(double (*a)()) {
  input = a;
}

void PID::getOutputFunction(double (*a)()) {
  output = a;
}

void PID::onControlSignal(void (*a)(double c)) {
  control = a;
}

