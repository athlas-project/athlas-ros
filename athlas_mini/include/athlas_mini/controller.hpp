//generate controller object

class Controller{    
  public:
  
    /* pid controller constructor
     * INPUT:
     * double p: Kp
     * double i: Ki
     * double d: Kd
     * int   ta: timestep [ms]  
     */
    Controller(double p,double i,double d,int ta);

    Controller();

    void setTa(int ta);

    void setKp(double p);

    void setKi(double i);

    void setKd(double d);

    /*
     * Change Windup state
     * INPUT
     * bool windup , true if actuator limited
     */
    void setWindup(bool windup);
    
    /* Calculate pid control
     * 
     * INPUT:
     * double r: reference value
     * double x: system state to track reference value
     * 
     * OUTPUT:
     * double System input to track reference value
     */
    double trackReference(double r, double x);

  private:
    double Kp,Ki,Kd;
    double i_sum;
    double e_old;
    bool antiWindup;
    int Ta;    
};



