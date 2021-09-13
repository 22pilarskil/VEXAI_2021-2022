class PD
{
  public:
  // PD preset variables
  
    double kp;
    double kd;
    double min_speed;
    
    // variables for each calculation
    
    double prev_error;
    int prev_time;
    
    // PD constructor and method
    
    PD(double p, double d, double min = 0);
    double getValue(double error);
   
}
