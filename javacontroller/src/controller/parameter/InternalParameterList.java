package controller.parameter;

import java.util.Collections;
import java.util.Comparator;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.LinkedList;




public class InternalParameterList extends Hashtable<String, InternalParameter>{

			
	public void addParameter(InternalParameter  param){
    	put(param.getKey(),param); 
    }
	
	
	public void setParameter(String key, Object o){
		try{
			get(key).setValue(o);
		}catch(Exception e){
			System.err.println("Parameter "+key+" can not be set on List");
		}
	}
    
    public void delParameter(InternalParameter param){
    	try{
    		remove(param);
    	}catch(Exception e){
			System.err.println("Parameter "+param.getKey()+" can not be removed");
		}
    }
    
    
    public String getAllNames(){
    	LinkedList<InternalParameter> list = new LinkedList<InternalParameter>(this.values());
    	Collections.sort(list,new SortList());
       	Iterator<InternalParameter> it = list.iterator();
    	String out = "";
       	
    	while(it.hasNext()){
    		InternalParameter p = it.next();
    		out+=p.getNameAsString();
    	}
    	
    	return out+"#$";
    }
    
    public String getAllValues(){
    	LinkedList<InternalParameter> list = new LinkedList<InternalParameter>(this.values());
    	Collections.sort(list,new SortList());
       	Iterator<InternalParameter> it = list.iterator();
    	String out = "";
       	while(it.hasNext()){
    		InternalParameter p = it.next();
    		out+=p.getValueAsString();
    	}
    	
    	return out;
    }
    
    
    
    
    
}

 class SortList implements Comparator<InternalParameter>{
	public int compare(InternalParameter o1, InternalParameter o2) {
		return o1.getKey().compareTo(o2.getKey());
	}
 }
