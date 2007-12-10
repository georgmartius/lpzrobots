package controller.parameter;

public class ConfigurableParameter{
	
	String key;
	double value;
		
	public ConfigurableParameter(String key, double value) {
		this.key = key;
		this.value = value;
	}
	
	
	
	public void setValue(double v){
		value = v;
	}
	
	public String getKey() {
		return key;
	}
	
	
	public String getValueAsString(){
		return String.valueOf(value);
	}
	
	
	public double getValueAsDouble(){
		return value;
	}
	
	public int getValueAsInt(){
		return (int)Math.round(value);
		
	}

}
