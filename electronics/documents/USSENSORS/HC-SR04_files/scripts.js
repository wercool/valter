function JQerrorInint(){jQuery.error=function(e){_gaq.push(["_trackEvent","jQuery Error",e,navigator.userAgent])}}function loadPopup(e){if(popupStatus==0){setWiH();$("#backgroundPopup").css({opacity:"0.7"});$("#backgroundPopup").fadeIn("slow");$("#popupBasket").fadeIn("slow");popupStatus=1;$("#popupBasket").html("<iframe class='popup' src='"+e+"' height='"+ifrh+"' width='"+ifrw+"' id='basketifr' style='padding:0; margin:0' scrolling='no' onload=\"setTimeout(setWiH, 10)\">��������...</iframe>")}}function setWiH(){$("#basketifr").height(ifrh);$("#basketifr").width(ifrw);$("#popupBasket").width(ifrw);$("#popupBasket").height(ifrh);centerPopup()}function disablePopup(){if(popupStatus==1){$("#backgroundPopup").fadeOut("slow");$("#popupBasket").fadeOut("slow");popupStatus=0;ifrh=350;ifrw=600}}function centerPopup(){var e=document.documentElement.clientWidth;var t=document.documentElement.clientHeight;var n=$("#popupBasket").height();var r=$("#popupBasket").width();if(t>n){$("#popupBasket").css({position:"fixed",top:t/2-n/2+"px",left:e/2-r/2})}else{$("#popupBasket").css({position:"absolute",top:"20px",left:e/2-r/2});jQuery("html:not(:animated),body:not(:animated)").animate({scrollTop:"body"},1100)}$("#backgroundPopup").css({height:t})}function opencart(e){centerPopup();loadPopup(e)}function disablePopupQS(){document.getElementById("backgroundPopup").style.display="none";document.getElementById("searchsuggestdiv").style.display="none";document.getElementById("searchsuggestdiv").innerHTML=""}function open_window(e,t,n){var r="width="+t+",height="+n+",menubar=no,location=no,resizable=yes,scrollbars=yes";newWin=window.open(e,"newWin",r);newWin.focus()}function open_printable_version(e){var t="menubar=no,location=no,resizable=yes,scrollbars=yes";newWin=window.open(e,"perintableWin",t);newWin.focus()}function confirmDelete(e,t,n){temp=window.confirm(t);if(temp){window.location=n+e}}function confirmUnsubscribe(){temp=window.confirm("{/literal}{$smarty.const.QUESTION_UNSUBSCRIBE}{literal}");if(temp){window.location="index.php?killuser=yes"}}function validate(){if(document.subscription_form.email.value.length<1){alert("{/literal}{$smarty.const.ERROR_INPUT_EMAIL}{literal}");return false}if(document.subscription_form.email.value=="Email"){alert("{/literal}{$smarty.const.ERROR_INPUT_EMAIL}{literal}");return false}return true}function validate_disc(){if(document.formD.nick.value.length<1){alert("{/literal}{$smarty.const.ERROR_INPUT_NICKNAME}{literal}");return false}if(document.formD.topic.value.length<1){alert("{/literal}{$smarty.const.ERROR_INPUT_MESSAGE_SUBJECT}{literal}");return false}document.formD.vert.value="2";return true}function validate_search(){if(document.Sform.price1.value!=""&&(document.Sform.price1.value<0||isNaN(document.Sform.price1.value))){alert("{/literal}{$smarty.const.ERROR_INPUT_PRICE}{literal}");return false}if(document.Sform.price2.value!=""&&(document.Sform.price2.value<0||isNaN(document.Sform.price2.value))){alert("{/literal}{$smarty.const.ERROR_INPUT_PRICE}{literal}");return false}return true}function ajaxCart(e,t,n,r){var i="";if(r)var s=$("input[name^='option_select_hidden_']").filter("input[name$='_"+e+"']");else var s=$("input[name^='option_select_hidden_']");for(var o=0;o<s.length;o++){i+="&"+s[o].name+"="+s[o].value}$.ajax({url:"ajaxCart.php",data:"productID="+e+"&numgoods="+t.numgoods.value+i,dataType:"text",type:"POST",success:function(e){$("#cart_informer").html(e)}});var u=$("#"+n).offset();var a=$("#cart_informer").offset();var f=u.left-a.left;var l=u.top-a.top;var c=Math.round(Math.sqrt(f*f+l*l));$("#"+n).find(".image").effect("transfer",{to:$("#cart_informer"),className:"transfer_class"},c);$(".transfer_class").html($("#"+n).html());$(".transfer_class").find("img").css("height","100%");return false}function addinfotocopytext(e){var t="\n\n<br /><br />�������� "+window.location.href+"<br /><br />\n\n";var n=25;t=t.replace(/(&)sid=[0-9a-f]{32}/g,"");var r=getElement("ctrlcopy");if(r!=null){r.parentNode.removeChild(r)}var r=document.createElement("div");r.id="ctrlcopy";r.style.position="absolute";r.style.left="-99999px";r.innerHTML=t;if(window.getSelection){var i=window.getSelection();var s=i.toString();if(!s||s.length<n)return;var o=i.getRangeAt(0);s=o.cloneRange();s.collapse(false);s.insertNode(r);o.setEndAfter(r);i.removeAllRanges();i.addRange(o)}else if(document.selection){var i=document.selection;var o=i.createRange();var s=o.text;if(!s||s.lengt<n)return;s=o.duplicate();s.collapse(false);s.pasteHTML(r.outerHTML);o.setEndPoint("EndToEnd",s);o.select()}}var popupStatus=0;var ifrh=370;var ifrw=600;$(document).ready(function(){$("#backgroundPopup").click(function(){disablePopup()});$(document).keypress(function(e){if(e.keyCode==27&&popupStatus==1){disablePopup();disablePopupQS()}})});if(typeof addEvent!=="function"){function addEvent(e,t,n,r){t=t.replace(/^(on)?/,"on");var i=e[t];var s="__tmp";e[t]=function(t){if(!t)t=window.event;var o;if(!r){e[s]=n;o=e[s](t);e[s]=null;if(o===false)return o}if(i){e[s]=i;o=e[s](t);e[s]=null}if(r&&o!==false){e[s]=n;o=e[s](t);e[s]=null}return o};return n}}if(typeof getElement!=="function"){function getElement(e){if(document.getElementById){return document.getElementById(e)}if(document.all){return document.all[e]}if(document.layers){return document.layers[e]}return null}}