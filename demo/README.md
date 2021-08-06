# Demo Data
In this folder are demo data files of the FBG needle shape compared with a ground truth

## FBG-Cam-Data_prediction_FBG-weights_needle_3CH_4AA_v2_04-InsExpmt-04-12-2021.mat  
This is a file that contains camera (ground truth) and FBG-determined shape along with prediction results in several tables.


The camera shape is generated from computer vision methods and is rotated back into the needle frame by performing a point cloud
registration between the FBG shape and the camera shapes. They are first aligned by the tips to be coincident since the computer vision method
reliably segments out the needle tip and are optimally rotated in order to align the FBG needle shape to the camera coordinates. This 
rigid body transformation is then inverted to rotate the ground truth into the needle frame.

The tables in this file are:
* `act_result_tbl`: A table of the non-predicted insertion shapes 
	* `Ins_hole`: the insertion hole of the Insertion jig
	* `L_ref`: the insertion depth of the needle
	* `cam_shape`: 3 x N arrays of the camera shape in needle coordinates
	* `fbg_shape`: 3 x M arrays of the FBG shape in needle coordinates
	* `RMSE`: RMSE of the alignment between `cam_shape` and `fbg_shape`
	* `MaxError`: the maximum error between `cam_shape` and `fbg_shape`
* `act_result_summ_Lref`: A table summarizing `act_result_tbl` by grouping `L_ref`
* `act_result_summ_InsHole`: A table summarizing `act_result_tbl` by grouping `Ins_Hole`
* `pred_result_tbl`: A table of the predicted insertion shapes
	* `Ins_hole`: the insertion hole of the Insertion jig
	* `L_ref`: the reference insertion depth of the needle
	* `L_pred`: the predicted insertion depth of the needle
	* `cam_shape`: 3 x N arrays of the camera shape in needle coordinates of the predicted insertion depth
	* `fbgref_shape`: 3 x M arrays of the FBG shape in needle coordinates the reference insertion shape at the predicted insertion depth
	* `pred_shape`: 3 x M arrays of the FBG shape in needle coordinates the predicted insertion shape from the reference insertion depth
	* `q_optim`: the optimal $$q$$ parameter used for the prediction
	* `FBG_RMSE`: the RMSE between the predicted and reference FBG shapes
	* `FBG_MaxError`: the maximum error between the predicted and reference FBG shapes
	* `FBG_TipError`: the error in the tip location between the predicted and reference FBG shapes
	* `Cam_RMSE`: the RMSE between the predicted FBG and reference Cam shapes
	* `Cam_MaxError`: the maximum error between the predicted FBG and reference Cam shapes
	* `Cam_TipError`: the error in the tip location between the predicted FBG and reference Cam shapes
* `pred_result_summ_Lref`: A table summarizing `pred_result_tbl` by grouping `L_ref`
* `pred_result_summ_Lpred`: A table summarizing `pred_result_tbl` by grouping `L_pred`
* `pred_result_summ_InsHole`: A table summarizing `pred_result_tbl` by grouping `Ins_Hole`
