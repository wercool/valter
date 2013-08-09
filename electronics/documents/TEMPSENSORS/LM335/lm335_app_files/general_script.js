//===============================
//		GetImgRatio()

// GetImgRatio() figures the ratio of the image (i.e. constrains to "base")
//	recieves an array duple, and a base size "sizeBase"; returns a duple (x,y)
function GetImgRatio(imgObj, sizeBase){
	
	// figure the image size
	var ratioTemp = imgObj.width/imgObj.height;
	if (ratioTemp==1){//sides are equal
		newWidth=sizeBase;
		newHeight=sizeBase;
	}else if (ratioTemp > 1){//width greater
		newWidth=sizeBase;
		newHeight=Math.round(sizeBase/ratioTemp);
	}else if (ratioTemp < 1){//height greater
		newWidth=Math.round(sizeBase*ratioTemp);
		newHeight=sizeBase;
	}
	
	imgSizeNew = new Array(newWidth, newHeight);
	return imgSizeNew;
	
} 
// <-- END GetImgRatio()
//===============================

function rollOver(callType, callProp1){
	document.body.style.cursor='pointer';
	
	//alert(callType+","+callProp1);
	
	switch(callType){
	// image swap
	case "imgSwap":
		// image swap
		targetObj = callProp1;
		if(targetObj != 'undefined'){
			if(targetObj.src.indexOf('_b') == "-1"){
				// add a "_b" to the end of the filename
				var src_temp = targetObj.src.split(".");
				src_temp[(src_temp.length)-2] = src_temp[(src_temp.length)-2]+"_b";
				src_temp = src_temp.join(".");
			}else{
				// take "_b" from the end of the filename
				var src_temp = targetObj.src.split(".");
				src_temp[(src_temp.length)-2] = src_temp[(src_temp.length)-2].replace(/_b/,"");
				src_temp = src_temp.join(".");
			}
			// flip the image
			targetObj.src = src_temp;
		}
		break;
	}
}

function rollOut(callType, callProp1){
	document.body.style.cursor='auto';
	
	switch(callType){
	// image swap
	case "imgSwap":
		targetObj = callProp1;
		if(targetObj != 'undefined'){
			if(targetObj.src.indexOf('_b') != "-1"){
				// take "_b" from the end of the filename
				var src_temp = targetObj.src.split(".");
				src_temp[(src_temp.length)-2] = src_temp[(src_temp.length)-2].replace(/_b/,"");
				src_temp = src_temp.join(".");
				// flip the image
				targetObj.src = src_temp;
			}
		}
		break;
	}
}
			
