/*  dev : Danu andrean 
 *  kalman filter
 *  year 2021
 */


float _err_measure;
float _err_estimate;
float _q;
float _current_estimate;
float _last_estimate;
float _kalman_gain;

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SetKalmanFilter(2, 2, 0.01);

}

void loop() {
  float real_value = 0.5;
  // add a noise to the reference value and use as the measured value
  float measured_value = real_value + random(-100,100)/100.0;

  // calculate the estimated value with Kalman Filter
  float estimated_value = UpdateEstimate(measured_value);

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > refresh_time) {
    Serial.print(real_value,4);
    Serial.print(",");
    Serial.print(measured_value,4);
    Serial.print(",");
    Serial.print(estimated_value,4);
    Serial.println();
    
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

}
float SetKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}
float UpdateEstimate(float mea)
{
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;

  return _current_estimate;
}
