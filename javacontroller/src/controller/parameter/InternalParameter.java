package controller.parameter;

import Jama.Matrix;

public class InternalParameter {

	private String key;

	private Object value;

	private int object_type = 0;

	private int print_type = 0;

	private static final int TYPE_INTEGER = 1;

	private static final int TYPE_INTEGER_ARRAY_1D = 11;

	private static final int TYPE_INTEGER_ARRAY_2D = 111;

	private static final int TYPE_DOUBLE = 2;

	private static final int TYPE_DOUBLE_ARRAY_1D = 22;

	private static final int TYPE_DOUBLE_ARRAY_2D = 222;

	private static final int TYPE_MATRIX = 3;

	
	public static final int Diagonal_and_4x4 = 1;

	public static final int MatrixNorm_MaxColSum = 2;

	public static final int MatrixNorm_MaxSingularValue = 3;

	public static final int MatrixNorm_MaxRowSum = 4;

	public static final int MatrixNorm_Frobenius = 5;
	

	public InternalParameter(String key, Object o) {

		this.key = key;
		this.value = o;

		String check = o.getClass().getSimpleName();

		if (check.equals("Integer"))
			object_type = TYPE_INTEGER;
		if (check.equals("int[]"))
			object_type = TYPE_INTEGER_ARRAY_1D;
		if (check.equals("int[][]"))
			object_type = TYPE_INTEGER_ARRAY_2D;

		if (check.equals("double"))
			object_type = TYPE_DOUBLE;
		if (check.equals("double[]"))
			object_type = TYPE_DOUBLE_ARRAY_1D;
		if (check.equals("double[][]"))
			object_type = TYPE_DOUBLE_ARRAY_2D;

		if (check.equals("Matrix"))
			object_type = TYPE_MATRIX;

	}
	
	public InternalParameter(String key, Matrix m, int print_type) {

		this.key = key;
		this.value = m;
		this.print_type = print_type;

		this.object_type = TYPE_MATRIX;

	}

	public String getKey() {
		return key;
	}

	public void setKey(String key) {
		this.key = key;
	}

	public Object getValue() {
		return value;
	}

	public void setValue(Object value) {
		this.value = value;
	}

	public String getNameAsString() {

		// int, double
		if (object_type == TYPE_INTEGER || object_type == TYPE_DOUBLE) {
			return "#" + String.valueOf(key);
		}

		// int[]
		if (object_type == TYPE_INTEGER_ARRAY_1D) {
			int l = ((int[]) value).length;
			String out = "";
			for (int i = 0; i < l; i++) {
				out += "#" + key + "[" + i + "]";
			}
			return String.valueOf(out);
		}

		// int[][]
		if (object_type == TYPE_INTEGER_ARRAY_2D) {
			int[][] d = (int[][]) value;
			int h = d.length;
			int v = d[0].length;
			String out = "";
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < v; j++) {
					try {
						out += "#" + key + "[" + i + "][" + j + "]";
					} catch (Exception e) {
					}
				}

			}
			return String.valueOf(out);
		}

		// double[]
		if (object_type == TYPE_DOUBLE_ARRAY_1D) {
			int l = ((double[]) value).length;
			String out = "";
			for (int i = 0; i < l; i++) {
				out += "#" + key + "[" + i + "]";
			}
			return String.valueOf(out);
		}

		// double[][]
		if (object_type == TYPE_DOUBLE_ARRAY_2D) {
			double[][] d = (double[][]) value;
			int h = d.length;
			int v = d[0].length;
			String out = "";
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < v; j++) {
					try {
						out += "#" + key + "[" + i + "][" + j + "]";
					} catch (Exception e) {
					}
				}

			}
			return String.valueOf(out);
		}

		
		//***************************************
		//Matrix
		if(object_type == TYPE_MATRIX){
			Matrix matrix = (Matrix)value;
			String out = "";
			
			//ganze matrix ausgeben##############
			if(print_type == 0){
				for(int i=0; i < matrix.getRowDimension();i++){
					for(int j=0; j < matrix.getColumnDimension(); j++){
						out+="#"+key+"[" + i + "][" + j + "]";
					}
				}
				return out;
			}//##################################
			
			//Diagonal_and_4x4###################
			if(print_type == Diagonal_and_4x4){
				int smallDimM = Math.min(matrix.getRowDimension(),4);
				int smallDimN = Math.min(matrix.getColumnDimension(),4);
				int smallerDim = Math.min(matrix.getRowDimension(),matrix.getColumnDimension());
				//4x4
				for(int i=0; i < smallDimM;i++){
					for(int j=0; j < smallDimN; j++){
						out+="#"+key+"[" + i + "][" + j + "]";
					}
				}
				
				//diagonale nach der 4x4 Matrix
				for(int i = 4; i < smallerDim; i++){
					out+="#"+key+"[" + i + "][" + i + "]";
				}
				
				
			return out;	
			}//##################################
			
			
			//fuer alle anderen matrix_print_typen
			return "#" + String.valueOf(key);
		}
		//***************************************
	
		
		return "#" + String.valueOf(key);

	}

	public String getValueAsString() {

		// int, double
		if (object_type == TYPE_INTEGER || object_type == TYPE_DOUBLE) {
			return "#" + String.valueOf(value);
		}

		// int[]
		if (object_type == TYPE_INTEGER_ARRAY_1D) {
			int[] d = (int[]) value;
			int l = d.length;
			String out = "";
			for (int i = 0; i < l; i++) {
				out += "#" + d[i];
			}
			return String.valueOf(out);
		}

		// int[][]
		if (object_type == TYPE_INTEGER_ARRAY_2D) {
			int[][] d = (int[][]) value;
			int h = d.length;
			int v = d[0].length;
			String out = "";
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < v; j++) {
					try {
						out += "#" + d[i][j];
					} catch (Exception e) {
					}
				}

			}
			return String.valueOf(out);
		}

		// double[]
		if (object_type == TYPE_DOUBLE_ARRAY_1D) {
			double[] d = (double[]) value;
			int l = d.length;
			String out = "";
			for (int i = 0; i < l; i++) {
				out += "#" + d[i];
			}
			return String.valueOf(out);
		}

		// double[][]
		if (object_type == TYPE_DOUBLE_ARRAY_2D) {
			double[][] d = (double[][]) value;
			int h = d.length;
			int v = d[0].length;
			String out = "";
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < v; j++) {
					try {
						out += "#" + d[i][j];
					} catch (Exception e) {
					}
				}

			}
			return String.valueOf(out);
		}
		
		
		//***************************************
		//Matrix
		if(object_type == TYPE_MATRIX){
			Matrix matrix = (Matrix)value;
			String out = "";
			
			//ganze matrix ausgeben###############	
			if(print_type == 0){
				for(int i=0; i < matrix.getRowDimension();i++){
					for(int j=0; j < matrix.getColumnDimension(); j++){
						out+="#"+ matrix.get(i, j);
					}
				}
				return out;
			}//##################################
			
			
			//Diagonal_and_4x4###################
			if(print_type == Diagonal_and_4x4){
				int smallDimM = Math.min(matrix.getRowDimension(),4);
				int smallDimN = Math.min(matrix.getColumnDimension(),4);
				int smallerDim = Math.min(matrix.getRowDimension(), matrix.getColumnDimension());
				//4x4
				for(int i=0; i < smallDimM;i++){
					for(int j=0; j < smallDimN; j++){
						out+="#"+ matrix.get(i, j);
					}
				}
				
				//diagonale nach der 4x4 Matrix
				for(int i = 4; i < smallerDim; i++){
					out+="#"+ matrix.get(i, i);
				}
				
				
			return out;	
			}//##################################
			
			
			//fuer alle anderen matrix_print_typen
			if(print_type == MatrixNorm_MaxColSum){
				return "#" + String.valueOf(matrix.norm1());			
			}
			
			if(print_type == MatrixNorm_MaxRowSum){
				return "#" + String.valueOf(matrix.normInf());			
			}
			
			if(print_type == MatrixNorm_MaxSingularValue){
				return "#" + String.valueOf(matrix.norm2());			
			}
			
			if(print_type == MatrixNorm_Frobenius){
				return "#" + String.valueOf(matrix.normF());			
			}
		}
		//***************************************

		return "#" + String.valueOf(value);
	}

}
