package controller.parameter;

import java.util.Hashtable; 

public class ConfigurableParameterList extends Hashtable{

	private static final long serialVersionUID = 1L;
		
	public void addParameter(ConfigurableParameter param){
    	put(param.getKey(),param);
    }
    
    public void delParameter(ConfigurableParameter param){
    	remove(param.getKey());
    }
    
    
    public ConfigurableParameter getParameter(Object Key){
    	if(get(Key) == null){
    		return null;
    	}
    	return (ConfigurableParameter)get(Key);
    }
    
	
	
	public void setParameter(Object Key,double v) {
		try{
			((ConfigurableParameter)get(Key)).setValue(v);
		}catch(Exception e){System.err.println("Parameter kann nicht gesetzt werden,weil nicht initialisiert");}
		
	}
	
	public void setParameter(Object Key,int v) {
		try{
			((ConfigurableParameter)get(Key)).setValue(v);
		}catch(Exception e){System.err.println("Parameter kann nicht gesetzt werden,weil nicht initialisiert");}
		
	}

}
