#include "encode.h"
#include "iostream" 
#include "../firmware/easylogging++.h"
#include <opencv2/highgui.hpp>

INITIALIZE_EASYLOGGINGPP
DF_Encode::DF_Encode()
{
}


DF_Encode::~DF_Encode()
{
}

bool DF_Encode::unwrapBase2Kmap(cv::Mat wrap_map, cv::Mat k1_map, cv::Mat k2_map, cv::Mat& unwrap_map)
{
	if (wrap_map.empty() || k1_map.empty() || k2_map.empty())
	{
		return false;
	}
	int nr = wrap_map.rows;
	int nc = wrap_map.cols;


	cv::Mat unwrap(nr, nc, CV_32F, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k1 = k1_map.ptr<ushort>(r);
		ushort* ptr_k2 = k2_map.ptr<ushort>(r);
		float* ptr_wrap = wrap_map.ptr<float>(r);
		float* ptr_unwrap = unwrap.ptr<float>(r);

		for (int c = 0; c < nc; c++)
		{

			int k2 = (ptr_k2[c] + 1) / 2;

			if (ptr_wrap[c] < CV_PI / 2.0)
			{
				ptr_unwrap[c] = ptr_wrap[c] + 2 * CV_PI * k2;
			}
			else if (ptr_wrap[c] < 3 * CV_PI / 2.0)
			{

				ptr_unwrap[c] = ptr_wrap[c] + 2 * CV_PI * ptr_k1[c];
			}
			else
			{
				ptr_unwrap[c] = ptr_wrap[c] + 2 * CV_PI * (k2 - 1);
			}
		}


	}

	unwrap_map = unwrap.clone();


	return false;
}

bool DF_Encode::computePhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& confidence_map, cv::Mat& global_map,
	cv::Mat& average_map, cv::Mat& brightness_map, cv::Mat& mask_map)
{
	if (patterns.empty())
	{
		return false;
	}

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat wrap(nr, nc, CV_32F, cv::Scalar(0));
	cv::Mat confidence(nr, nc, CV_32F, cv::Scalar(0));
	cv::Mat average(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat brightness(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat global(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));

	switch (patterns.size())
	{
	case 6:
	{
#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{
			uchar* ptr0 = patterns[0 + 3].ptr<uchar>(r);
			uchar* ptr1 = patterns[1 + 3].ptr<uchar>(r);
			uchar* ptr2 = patterns[2 + 3].ptr<uchar>(r);
			uchar* ptr3 = patterns[3 - 3].ptr<uchar>(r);
			uchar* ptr4 = patterns[4 - 3].ptr<uchar>(r);
			uchar* ptr5 = patterns[5 - 3].ptr<uchar>(r);

			uchar* ptr_m = mask.ptr<uchar>(r);
			uchar* ptr_avg = average.ptr<uchar>(r);
			uchar* ptr_b = brightness.ptr<uchar>(r);
			uchar* ptr_g = global.ptr<uchar>(r);
			float* ptr_con = confidence.ptr<float>(r);
			float* ptr_wrap = wrap.ptr<float>(r);

			for (int c = 0; c < nc; c++)
			{
				int exposure_num = 0;

				if (255 == ptr0[c])
				{
					exposure_num++;
				}
				if (255 == ptr1[c])
				{
					exposure_num++;
				}
				if (255 == ptr2[c])
				{
					exposure_num++;
				}
				if (255 == ptr3[c])
				{
					exposure_num++;
				}
				if (255 == ptr4[c])
				{
					exposure_num++;
				}
				if (255 == ptr5[c])
				{
					exposure_num++;
				}


				float b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
					+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);

				float a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
					+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

				float ave = (ptr0[c] + ptr1[c] + ptr2[c] + ptr3[c] + ptr4[c] + ptr5[c]) / 6.0;

				float r = std::sqrt(a * a + b * b);

				ptr_avg[c] = ave + 0.5;
				ptr_con[c] = r / 3.0;
				ptr_b[c] = ave + 0.5 + r / 3.0;
				ptr_g[c] = ave + 0.5 - r / 3.0;
				/***********************************************************************/

				if (exposure_num > 3)
				{
					ptr_m[c] = 0;
					ptr_wrap[c] = -1;
				}
				else
				{
					ptr_wrap[c] = CV_PI + std::atan2(a, b);
				}


			}
		}
	}
	break;
	default:
		break;
	}




	/*****************************************************************************************************************************/

	confidence_map = confidence.clone();
	wrap_map = wrap.clone();
	brightness_map = brightness.clone();
	average_map = average.clone();
	mask_map = mask.clone();
	global_map = global.clone();

	return true;
}

bool DF_Encode::computePhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& confidence_map, cv::Mat& average_map, cv::Mat& brightness_map, cv::Mat& mask_map)
{
	if (patterns.empty())
	{
		return false;
	}

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat wrap(nr, nc, CV_32F, cv::Scalar(0));
	cv::Mat confidence(nr, nc, CV_32F, cv::Scalar(0));
	cv::Mat average(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat brightness(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat global(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));

	switch (patterns.size())
	{
	case 6:
	{
#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{
			uchar* ptr0 = patterns[0 + 3].ptr<uchar>(r);
			uchar* ptr1 = patterns[1 + 3].ptr<uchar>(r);
			uchar* ptr2 = patterns[2 + 3].ptr<uchar>(r);
			uchar* ptr3 = patterns[3 - 3].ptr<uchar>(r);
			uchar* ptr4 = patterns[4 - 3].ptr<uchar>(r);
			uchar* ptr5 = patterns[5 - 3].ptr<uchar>(r);

			uchar* ptr_m = mask.ptr<uchar>(r);
			uchar* ptr_avg = average.ptr<uchar>(r);
			uchar* ptr_b = brightness.ptr<uchar>(r);
			uchar* ptr_g = global.ptr<uchar>(r);
			float* ptr_con = confidence.ptr<float>(r);
			float* ptr_wrap = wrap.ptr<float>(r);

			for (int c = 0; c < nc; c++)
			{
				int exposure_num = 0;

				if (255 == ptr0[c])
				{
					exposure_num++;
				}
				if (255 == ptr1[c])
				{
					exposure_num++;
				}
				if (255 == ptr2[c])
				{
					exposure_num++;
				}
				if (255 == ptr3[c])
				{
					exposure_num++;
				}
				if (255 == ptr4[c])
				{
					exposure_num++;
				}
				if (255 == ptr5[c])
				{
					exposure_num++;
				}


				float b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
					+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);

				float a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
					+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

				float ave = (ptr0[c] + ptr1[c] + ptr2[c] + ptr3[c] + ptr4[c] + ptr5[c]) / 6.0;

				float r = std::sqrt(a * a + b * b);

				ptr_avg[c] = ave + 0.5;
				ptr_con[c] = r / 3.0;
				ptr_b[c] = ave + 0.5 + r / 3.0;
				ptr_g[c] = ave + 0.5 - r / 3.0;
				/***********************************************************************/

				if (exposure_num > 3)
				{
					ptr_m[c] = 0;
					ptr_wrap[c] = -1;
				}
				else
				{
					ptr_wrap[c] = CV_PI + std::atan2(a, b);
				}


			}
		}
	}
	break;
	default:
		break;
	}




	/*****************************************************************************************************************************/

	confidence_map = confidence.clone();
	wrap_map = wrap.clone();
	brightness_map = brightness.clone();
	average_map = average.clone();
	mask_map = mask.clone();

	return true;
}


bool DF_Encode::grayCodeToXorCode(std::vector<cv::Mat> gray_code, std::vector<cv::Mat>& xor_code)
{
	if (gray_code.empty())
	{
		return false;
	}

	xor_code.clear();


	cv::Mat template_pattern = gray_code.back().clone();
	int nr = template_pattern.rows;
	int nc = template_pattern.cols;

	for (int p = 0; p < gray_code.size() - 1; p++)
	{
		 
		cv::Mat pattern = gray_code[p].clone(); 

		cv::Mat merge_map(nr, nc, CV_8U, cv::Scalar(0));

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_0 = template_pattern.ptr<uchar>(r);
			uchar* ptr_1 = pattern.ptr<uchar>(r);
			uchar* ptr_m = merge_map.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{

				ptr_m[c] = ptr_0[c] ^ ptr_1[c];
			}


		}

		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 1));

		cv::morphologyEx(merge_map, merge_map, cv::MORPH_OPEN, element);
		cv::morphologyEx(merge_map, merge_map, cv::MORPH_CLOSE, element);
		 
		xor_code.push_back(merge_map.clone()); 
	}

	xor_code.push_back(template_pattern);
	 
	return true;
}


bool DF_Encode::minsw8CodeToValue(int minsw, int& value)
{
	if (minsw > 255)
	{
		return false;
	}

	value = -1;

	//std::vector<int> list{ 0, 1, 3, 7, 15, 31, 30, 62, 126, 254, 246, 247, 245, 213, 209, 145,
	//153, 152, 136, 8, 40, 42, 43, 35, 99, 103, 71, 70, 68, 76, 204, 220, 252, 253, 189, 185,
	//177, 179, 178, 146, 210, 82, 90, 91, 75, 107, 111, 109, 101, 100, 36, 164, 132, 134, 135,
	//143, 159, 155, 187, 186, 250, 242, 114, 112, 80, 81, 17, 21, 29, 13, 12, 44, 46, 174, 166,
	//167, 231, 199, 195, 193, 201, 200, 216, 88, 120, 56, 57, 49, 51, 55, 23, 22, 86, 94, 222,
	//206, 238, 239, 237, 233, 225, 161, 160, 128, 130, 2, 10, 11, 27, 59, 63, 127, 119, 118, 116,
	//244, 212, 148, 149, 157, 141, 137, 169, 168, 170, 162, 34, 98, 66, 67, 65, 69, 77, 93, 92,
	//124, 60, 188, 180, 181, 183, 151, 147, 211, 219, 218, 202, 74, 106, 104, 105, 97, 33, 37,
	//5, 4, 6, 14, 142, 158, 190, 191, 255, 251, 243, 241, 240, 208, 144, 16, 24, 25, 9, 41, 45,
	//47, 39, 38, 102, 230, 198, 196, 197, 205, 221, 217, 249, 248, 184, 176, 48, 50, 18, 19, 83,
	//87, 95, 79, 78, 110, 108, 236, 228, 229, 165, 133, 129, 131, 139, 138, 154, 26, 58, 122, 123,
	//115, 113, 117, 85, 84, 20, 28, 156, 140, 172, 173, 175, 171, 163, 227, 226, 194, 192, 64, 72,
	//73, 89, 121, 125, 61, 53, 52, 54, 182, 150, 214, 215, 223, 207, 203, 235, 234, 232, 224, 96, 32 };

	//auto iter = std::find(list.begin(), list.end(), minsw);

	//if (iter != list.end())
	//{
	//	value = iter - list.begin();
	//}
	//else
	//{
	//	return false;
	//}

	std::vector<int> list{ 0, 1, 105, 2, 155, 154, 156, 3, 19, 172, 106, 107, 70, 69, 157, 4, 169,
		66, 192, 193, 220, 67, 91, 90, 170, 171, 211, 108, 221, 68, 6, 5, 255, 152, 126, 23, 50, 
		153, 177, 176, 20, 173, 21, 22, 71, 174, 72, 175, 190, 87, 191, 88, 241, 240, 242, 89, 85,
		86, 212, 109, 136, 239, 7, 110, 233, 130, 128, 129, 28, 131, 27, 26, 234, 235, 147, 44, 29,
		132, 198, 197, 64, 65, 41, 194, 219, 218, 92, 195, 83, 236, 42, 43, 134, 133, 93, 196, 254,
		151, 127, 24, 49, 48, 178, 25, 149, 150, 148, 45, 200, 47, 199, 46, 63, 216, 62, 215, 114, 
		217, 113, 112, 84, 237, 213, 214, 135, 238, 8, 111, 103, 206, 104, 207, 52, 205, 53, 54, 
		18, 121, 209, 208, 223, 120, 158, 55, 168, 15, 39, 142, 117, 118, 244, 141, 17, 16, 210, 57,
		222, 119, 159, 56, 102, 101, 125, 228, 51, 204, 74, 75, 123, 122, 124, 227, 224, 225, 73, 
		226, 189, 36, 38, 37, 138, 139, 243, 140, 188, 35, 59, 58, 137, 34, 160, 161, 232, 79, 231,
		78, 181, 182, 180, 77, 81, 80, 146, 249, 30, 183, 95, 248, 167, 14, 40, 143, 116, 13, 245,
		246, 82, 185, 145, 144, 31, 184, 94, 247, 253, 100, 230, 229, 202, 203, 179, 76, 252, 99, 
		251, 250, 201, 98, 96, 97, 166, 165, 61, 164, 115, 12, 10, 11, 187, 186, 60, 163, 32, 33, 9, 162 };

	value = list[minsw];

	return true;


}

bool DF_Encode::minsw10CodeToValue(int minsw, int& value)
{
	if (minsw > 1023)
	{
		return false;
	}

	value = -1;

	std::vector<int> list{ 0   ,
1   ,
65  ,
81  ,
85  ,
341 ,
469 ,
501 ,
509 ,
1021,
957 ,
941 ,
937 ,
681 ,
553 ,
521 ,
513 ,
515 ,
579 ,
595 ,
599 ,
87  ,
215 ,
247 ,
255 ,
511 ,
447 ,
431 ,
427 ,
939 ,
811 ,
779 ,
771 ,
770 ,
834 ,
850 ,
854 ,
598 ,
726 ,
758 ,
766 ,
254 ,
252 ,
236 ,
232 ,
488 ,
424 ,
392 ,
384 ,
385 ,
257 ,
273 ,
277 ,
789 ,
853 ,
885 ,
893 ,
637 ,
765 ,
749 ,
745 ,
233 ,
169 ,
137 ,
129 ,
131 ,
3   ,
19  ,
23  ,
279 ,
343 ,
375 ,
383 ,
895 ,
1023,
1007,
1003,
747 ,
683 ,
651 ,
643 ,
642 ,
514 ,
530 ,
534 ,
22  ,
86  ,
118 ,
126 ,
382 ,
510 ,
494 ,
490 ,
1002,
938 ,
906 ,
898 ,
896 ,
768 ,
784 ,
788 ,
532 ,
596 ,
628 ,
636 ,
124 ,
125  ,
109  ,
105  ,
361  ,
489  ,
457  ,
449  ,
451  ,
387  ,
403  ,
407  ,
919  ,
791  ,
823  ,
831  ,
575  ,
639  ,
638  ,
634  ,
122  ,
250  ,
234  ,
226  ,
224  ,
160  ,
128  ,
132  ,
388  ,
260  ,
276  ,
284  ,
796  ,
860  ,
892  ,
888  ,
632  ,
760  ,
744  ,
736  ,
737  ,
673  ,
641  ,
645  ,
133  ,
5    ,
21   ,
29   ,
285  ,
349  ,
381  ,
377  ,
889  ,
1017 ,
1001 ,
993  ,
995  ,
931  ,
899  ,
903  ,
647  ,
519  ,
535  ,
543  ,
31   ,
30   ,
62   ,
58   ,
314  ,
378  ,
362  ,
354  ,
352  ,
480  ,
448  ,
452  ,
964  ,
900  ,
916  ,
924  ,
668  ,
540  ,
572  ,
568  ,
56   ,
120  ,
104  ,
96   ,
97   ,
225  ,
193  ,
197  ,
453  ,
389  ,
405  ,
413  ,
925  ,
797  ,
829  ,
825  ,
569  ,
633  ,
617  ,
609  ,
611  ,
739  ,
707  ,
711  ,
199  ,
135  ,
151  ,
159  ,
415  ,
287  ,
319  ,
315  ,
827  ,
891  ,
875  ,
867  ,
866  ,
994  ,
962  ,
966  ,
710  ,
646  ,
662  ,
670  ,
158  ,
156  ,
188  ,
184  ,
440  ,
312  ,
296  ,
288  ,
289  ,
353  ,
321  ,
325  ,
837  ,
965  ,
981  ,
989  ,
733  ,
669  ,
701  ,
697  ,
185  ,
57   ,
41   ,
33   ,
35   ,
99   ,
67   ,
71   ,
327  ,
455  ,
471  ,
479  ,
991  ,
927  ,
959  ,
955  ,
699  ,
571  ,
555  ,
547  ,
546  ,
610  ,
578  ,
582  ,
70   ,
198  ,
214  ,
222  ,
478  ,
414  ,
446  ,
442  ,
954  ,
826  ,
810  ,
802  ,
800  ,
864  ,
832  ,
836  ,
580  ,
708  ,
724  ,
732  ,
220  ,
221  ,
253  ,
249  ,
505  ,
441  ,
425  ,
417  ,
419  ,
291  ,
259  ,
263  ,
775  ,
839  ,
855  ,
863  ,
607  ,
735  ,
767  ,
763  ,
251  ,
187  ,
171  ,
163  ,
162  ,
34   ,
2    ,
6    ,
262  ,
326  ,
342  ,
350  ,
862  ,
990  ,
1022 ,
1018 ,
762  ,
698  ,
682  ,
674  ,
672  ,
544  ,
512  ,
516  ,
4    ,
68   ,
84   ,
92   ,
348  ,
476  ,
508  ,
504  ,
1016 ,
952  ,
936  ,
928  ,
929  ,
801  ,
769  ,
773  ,
517  ,
581  ,
597  ,
605  ,
93   ,
95   ,
127  ,
123  ,
379  ,
507  ,
491  ,
483  ,
482  ,
418  ,
386  ,
390  ,
902  ,
774  ,
790  ,
798  ,
542  ,
606  ,
604  ,
600  ,
88   ,
216  ,
248  ,
240  ,
241  ,
177  ,
161  ,
165  ,
421  ,
293  ,
261  ,
269  ,
781  ,
845  ,
861  ,
857  ,
601  ,
729  ,
761  ,
753  ,
755  ,
691  ,
675  ,
679  ,
167  ,
39   ,
7    ,
15   ,
271  ,
335  ,
351  ,
347  ,
859  ,
987  ,
1019 ,
1011 ,
1010 ,
946  ,
930  ,
934  ,
678  ,
550  ,
518  ,
526  ,
14   ,
12   ,
28   ,
24   ,
280  ,
344  ,
376  ,
368  ,
369  ,
497  ,
481  ,
485  ,
997  ,
933  ,
901  ,
909  ,
653  ,
525  ,
541  ,
537  ,
25   ,
89   ,
121  ,
113  ,
115  ,
243  ,
227  ,
231  ,
487  ,
423  ,
391  ,
399  ,
911  ,
783  ,
799  ,
795  ,
539  ,
603  ,
635  ,
627  ,
626  ,
754  ,
738  ,
742  ,
230  ,
166  ,
134  ,
142  ,
398  ,
270  ,
286  ,
282  ,
794  ,
858  ,
890  ,
882  ,
880  ,
1008 ,
992  ,
996  ,
740  ,
676  ,
644  ,
652  ,
140  ,
141  ,
157  ,
153  ,
409  ,
281  ,
313  ,
305  ,
307  ,
371  ,
355  ,
359  ,
871  ,
999  ,
967  ,
975  ,
719  ,
655  ,
671  ,
667  ,
155  ,
27   ,
59   ,
51   ,
50   ,
114  ,
98   ,
102  ,
358  ,
486  ,
454  ,
462  ,
974  ,
910  ,
926  ,
922  ,
666  ,
538  ,
570  ,
562  ,
560  ,
624  ,
608  ,
612  ,
100  ,
228  ,
196  ,
204  ,
460  ,
396  ,
412  ,
408  ,
920  ,
792  ,
824  ,
816  ,
817  ,
881  ,
865  ,
869  ,
613  ,
741  ,
709  ,
717  ,
205  ,
207  ,
223  ,
219  ,
475  ,
411  ,
443  ,
435  ,
434  ,
306  ,
290  ,
294  ,
806  ,
870  ,
838  ,
846  ,
590  ,
718  ,
734  ,
730  ,
218  ,
154  ,
186  ,
178  ,
176  ,
48   ,
32   ,
36   ,
292  ,
356  ,
324  ,
332  ,
844  ,
972  ,
988  ,
984  ,
728  ,
664  ,
696  ,
688  ,
689  ,
561  ,
545  ,
549  ,
37   ,
101  ,
69   ,
77   ,
333  ,
461  ,
477  ,
473  ,
985  ,
921  ,
953  ,
945  ,
947  ,
819  ,
803  ,
807  ,
551  ,
615  ,
583  ,
591  ,
79   ,
78   ,
94   ,
90   ,
346  ,
474  ,
506  ,
498  ,
496  ,
432  ,
416  ,
420  ,
932  ,
804  ,
772  ,
780  ,
524  ,
588  ,
589  ,
585  ,
73   ,
201  ,
217  ,
209  ,
211  ,
147  ,
179  ,
183  ,
439  ,
311  ,
295  ,
303  ,
815  ,
879  ,
847  ,
843  ,
587  ,
715  ,
731  ,
723  ,
722  ,
658  ,
690  ,
694  ,
182  ,
54   ,
38   ,
46   ,
302  ,
366  ,
334  ,
330  ,
842  ,
970  ,
986  ,
978  ,
976  ,
912  ,
944  ,
948  ,
692  ,
564  ,
548  ,
556  ,
44   ,
45   ,
13   ,
9    ,
265  ,
329  ,
345  ,
337  ,
339  ,
467  ,
499  ,
503  ,
1015 ,
951  ,
935  ,
943  ,
687  ,
559  ,
527  ,
523  ,
11   ,
75   ,
91   ,
83   ,
82   ,
210  ,
242  ,
246  ,
502  ,
438  ,
422  ,
430  ,
942  ,
814  ,
782  ,
778  ,
522  ,
586  ,
602  ,
594  ,
592  ,
720  ,
752  ,
756  ,
244  ,
180  ,
164  ,
172  ,
428  ,
300  ,
268  ,
264  ,
776  ,
840  ,
856  ,
848  ,
849  ,
977  ,
1009 ,
1013 ,
757  ,
693  ,
677  ,
685  ,
173  ,
175  ,
143  ,
139  ,
395  ,
267  ,
283  ,
275  ,
274  ,
338  ,
370  ,
374  ,
886  ,
1014 ,
998  ,
1006 ,
750  ,
686  ,
654  ,
650  ,
138  ,
10   ,
26   ,
18   ,
16   ,
80   ,
112  ,
116  ,
372  ,
500  ,
484  ,
492  ,
1004 ,
940  ,
908  ,
904  ,
648  ,
520  ,
536  ,
528  ,
529  ,
593  ,
625  ,
629  ,
117  ,
245  ,
229  ,
237  ,
493  ,
429  ,
397  ,
393  ,
905  ,
777  ,
793  ,
785  ,
787  ,
851  ,
883  ,
887  ,
631  ,
759  ,
743  ,
751  ,
239  ,
238  ,
206  ,
202  ,
458  ,
394  ,
410  ,
402  ,
400  ,
272  ,
304  ,
308  ,
820  ,
884  ,
868  ,
876  ,
620  ,
748  ,
716  ,
712  ,
200  ,
136  ,
152  ,
144  ,
145  ,
17   ,
49   ,
53   ,
309  ,
373  ,
357  ,
365  ,
877  ,
1005 ,
973  ,
969  ,
713  ,
649  ,
665  ,
657  ,
659  ,
531  ,
563  ,
567  ,
55   ,
119  ,
103  ,
111  ,
367  ,
495  ,
463  ,
459  ,
971  ,
907  ,
923  ,
915  ,
914  ,
786  ,
818  ,
822  ,
566  ,
630  ,
614  ,
622  ,
110  ,
108  ,
76   ,
72   ,
328  ,
456  ,
472  ,
464  ,
465  ,
401  ,
433  ,
437  ,
949  ,
821  ,
805  ,
813  ,
557  ,
621  ,
623  ,
619  ,
107  ,
235  ,
203  ,
195  ,
194  ,
130  ,
146  ,
150  ,
406  ,
278  ,
310  ,
318  ,
830  ,
894  ,
878  ,
874  ,
618  ,
746  ,
714  ,
706  ,
704  ,
640  ,
656  ,
660  ,
148  ,
20   ,
52   ,
60   ,
316  ,
380  ,
364  ,
360  ,
872  ,
1000 ,
968  ,
960  ,
961  ,
897  ,
913  ,
917  ,
661  ,
533  ,
565  ,
573  ,
61   ,
63   ,
47   ,
43   ,
299  ,
363  ,
331  ,
323  ,
322  ,
450  ,
466  ,
470  ,
982  ,
918  ,
950  ,
958  ,
702  ,
574  ,
558  ,
554  ,
42   ,
106  ,
74   ,
66   ,
64   ,
192  ,
208  ,
212  ,
468  ,
404  ,
436  ,
444  ,
956  ,
828  ,
812  ,
808  ,
552  ,
616  ,
584  ,
576  ,
577  ,
705  ,
721  ,
725  ,
213  ,
149  ,
181  ,
189  ,
445  ,
317  ,
301  ,
297  ,
809  ,
873  ,
841  ,
833  ,
835  ,
963  ,
979  ,
983  ,
727  ,
663  ,
695  ,
703  ,
191  ,
190  ,
174  ,
170  ,
426  ,
298  ,
266  ,
258  ,
256  ,
320  ,
336  ,
340  ,
852  ,
980  ,
1012 ,
1020 ,
764  ,
700  ,
684  ,
680  ,
168  ,
40   ,
8    ,
	};

	auto iter = std::find(list.begin(), list.end(), minsw);
	
	if (iter != list.end())
	{
		value = iter - list.begin();
	}
	else
	{
		return false;
	}

	return true;

}
 
bool DF_Encode::computeXOR05(std::vector<cv::Mat> patterns, cv::Mat& k1_map, cv::Mat& k2_map, cv::Mat& mask_map)
{

	if (patterns.empty())
	{
		return false;
	}

	cv::Mat black_map = patterns[patterns.size() - 2];
	cv::Mat white_map = patterns[patterns.size() - 1];

	int nr = black_map.rows;
	int nc = black_map.cols;


	cv::Mat threshold_map(nr, nc, CV_8U);
	cv::Mat mask(nr, nc, CV_8U);

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = black_map.ptr<uchar>(r);
		uchar* ptr_w = white_map.ptr<uchar>(r);
		uchar* ptr_t = threshold_map.ptr<uchar>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			float d = ptr_w[c] - ptr_b[c];
			ptr_t[c] = ptr_b[c] + 0.5 + d / 2.0;
			ptr_m[c] = 0.5 + d / 2.0;
		}

	}



	std::vector<cv::Mat> bin_xor_patterns;

	for (int p_i = 0; p_i < patterns.size()-2; p_i++)
	{
		cv::Mat bin_pattern(nr, nc, CV_8U, cv::Scalar(0));

		for (int r = 0; r < nr; r++)
		{ 
			uchar* ptr_p = patterns[p_i].ptr<uchar>(r);
			uchar* ptr_t = threshold_map.ptr<uchar>(r);
			uchar* ptr_bin = bin_pattern.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				if (ptr_p[c] < ptr_t[c])
				{
					ptr_bin[c] = 0;
				}
				else
				{
					ptr_bin[c] = 255;
				}

			}

		}



		bin_xor_patterns.push_back(bin_pattern.clone());
	}

	 
	std::vector<cv::Mat> gray_patterns; 
	grayCodeToXorCode(bin_xor_patterns, gray_patterns); 

	for (int i = 0; i < gray_patterns.size(); i++)
	{
		std::string path = "../gray_" + std::to_string(i) + ".bmp";
		cv::Mat img = gray_patterns[i].clone();
		cv::imwrite(path, img);
	}
	 
	decodeGrayCode(gray_patterns, threshold_map, k1_map, k2_map);

	//for (int p_i = 0; p_i < xor_patterns.size(); p_i++)
	//{

	//	std::string path = "G:\\DFX_CODE\\xema\\dev\\xema\\x64\\Release\\Patterns\\xor_";

	//	path += std::to_string(p_i);
	//	path += ".bmp";

	//	cv::imwrite(path, xor_patterns[p_i]);

	//	cv::Mat diff = patterns[p_i] - gray_patterns[p_i];
	//}

	mask_map = mask.clone();


	return true;
}

bool DF_Encode::grayCodeToBinCode(std::vector<bool> gray_code, std::vector<bool>& bin_code)
{
	if (gray_code.empty())
	{
		return false;
	}

	bin_code.push_back(gray_code[0]);

	for (int i = 1; i < gray_code.size(); i++)
	{
		bool val = bin_code[i - 1] ^ gray_code[i];
		bin_code.push_back(val);
	}

	return true;
}


bool DF_Encode::decodeMinswGrayCode(std::vector<cv::Mat> patterns, cv::Mat threshold, cv::Mat direct, cv::Mat global, cv::Mat& uncertain, cv::Mat& k_map)
{
	if (patterns.empty())
	{
		return false;
	}

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat bit_value_map(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat uncertain_map(nr, nc, CV_8U, cv::Scalar(0));

	//std::vector<std::vector<bool>> gray_code_list;
	//threshold bin
	std::vector<cv::Mat> bin_patterns;
	for (int i = 0; i < patterns.size(); i++)
	{
		cv::Mat bin_mat(nr, nc, CV_8U, cv::Scalar(0));
		int mv_i = (patterns.size() - i - 1);

		cv::Mat pattern = patterns[i].clone();
		cv::blur(pattern, pattern, cv::Size(5, 5));

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_bin = bin_mat.ptr<uchar>(r); 
			uchar* ptr_p = pattern.ptr<uchar>(r);
			uchar* ptr_val = bit_value_map.ptr<uchar>(r);
			uchar* ptr_direct = direct.ptr<uchar>(r);
			uchar* ptr_global = global.ptr<uchar>(r);
			uchar* ptr_threshold = threshold.ptr<uchar>(r);
			uchar* ptr_uncertain = uncertain_map.ptr<uchar>(r);


			for (int c = 0; c < nc; c++)
			{
				uchar val = ptr_val[c];

				uchar mask = 1 << mv_i;
				val = val & (~mask);
				uchar set_bit = 0;

				if (ptr_direct[c] < 2)
				{
					ptr_uncertain[c] = 20;
				}
				  
				if (ptr_direct[c] > ptr_global[c])
				{
					if (ptr_p[c] > ptr_direct[c])
					{
						ptr_bin[c] = 255;
						set_bit = 1 << mv_i;
					}
					else if (ptr_p[c] < ptr_global[c])
					{
						ptr_bin[c] = 0;
					}
					else
					{
						if (ptr_p[c] > ptr_threshold[c])
						{
							ptr_bin[c] = 255;
							set_bit = 1 << mv_i;
						}
						else
						{
							ptr_bin[c] = 0;
						}
						 

						ptr_uncertain[c] = 10;
					}
				}
				else
				{
					if (ptr_p[c] < ptr_direct[c])
					{
						ptr_bin[c] = 0;
					}
					else if (ptr_p[c] > ptr_global[c])
					{
						ptr_bin[c] = 255;
						set_bit = 1 << mv_i;
					}
					else
					{
						ptr_uncertain[c] = 255;
					}

				}





				//if (ptr_gray[c] < ptr_avg[c])
				//{
				//	ptr_bin[c] = 0;
				//}
				//else
				//{
				//	ptr_bin[c] = 255;
				//	set_bit = 1 << mv_i;
				//}


				val = val | set_bit;
				ptr_val[c] = val;

			}
		}
		bin_patterns.push_back(bin_mat.clone());
	}

	cv::Mat k1(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k1 = k1.ptr<ushort>(r);

		for (int c = 0; c < nc; c++)
		{
			std::vector<bool> gray_code_list;

			for (int i = 0; i < bin_patterns.size(); i++)
			{
				uchar val = bin_patterns[i].at<uchar>(r, c);

				if (255 == val)
				{
					gray_code_list.push_back(true);
				}
				else
				{
					gray_code_list.push_back(false);
				}
			}

			ushort k_1 = 0;
			for (int i = 0; i < gray_code_list.size(); i++)
			{
				k_1 += gray_code_list[i] * std::pow(2, gray_code_list.size() - i - 1);

			}
			ptr_k1[c] = k_1;
		}

	}

	k_map = k1.clone();

	uncertain = uncertain_map.clone();

	return true;

}

bool DF_Encode::decodeMinswGrayCode(std::vector<cv::Mat> patterns, std::vector<cv::Mat> patterns_inv,
	cv::Mat direct, cv::Mat global, cv::Mat& uncertain, cv::Mat& k_map)
{
	if (patterns.size() != patterns_inv.size() || patterns.empty())
	{
		return false;
	}

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat bit_value_map(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat uncertain_map(nr, nc, CV_8U, cv::Scalar(0));

	//std::vector<std::vector<bool>> gray_code_list;
	//threshold bin
	std::vector<cv::Mat> bin_patterns;
	for (int i = 0; i < patterns.size(); i++)
	{
		cv::Mat bin_mat(nr, nc, CV_8U, cv::Scalar(0));
		int mv_i = (patterns.size() - i - 1);

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_bin = bin_mat.ptr<uchar>(r);
			uchar* ptr_p_inv = patterns_inv[i].ptr<uchar>(r);
			uchar* ptr_p = patterns[i].ptr<uchar>(r);
			uchar* ptr_val = bit_value_map.ptr<uchar>(r);
			uchar* ptr_direct = direct.ptr<uchar>(r);
			uchar* ptr_global = global.ptr<uchar>(r);
			uchar* ptr_uncertain = uncertain_map.ptr<uchar>(r);


			for (int c = 0; c < nc; c++)
			{
				uchar val = ptr_val[c];

				uchar mask = 1 << mv_i;
				val = val & (~mask);
				uchar set_bit = 0;

				if (ptr_direct[c] < 2)
				{
					ptr_uncertain[c] = 255;
				}

				if (ptr_direct[c] > ptr_global[c])
				{
					if (ptr_p[c] > ptr_p_inv[c])
					{
						ptr_bin[c] = 255;
						set_bit = 1 << mv_i;
					}
					else if (ptr_p[c] < ptr_p_inv[c])
					{
						ptr_bin[c] = 0;
					}
					else
					{
						ptr_uncertain[c] = 255;
					}
				}
				else
				{
					if (ptr_p[c] < ptr_direct[c] && ptr_p_inv[c] > ptr_global[c])
					{
						ptr_bin[c] = 0;
					}
					else
					{
						ptr_uncertain[c] = 255;
					}

					if (ptr_p[c] > ptr_global[c] && ptr_p_inv[c] < ptr_direct[c])
					{
						ptr_bin[c] = 255;
						set_bit = 1 << mv_i;
					}
					else
					{
						ptr_uncertain[c] = 255;
					}

				}


				 


				//if (ptr_gray[c] < ptr_avg[c])
				//{
				//	ptr_bin[c] = 0;
				//}
				//else
				//{
				//	ptr_bin[c] = 255;
				//	set_bit = 1 << mv_i;
				//}


				val = val | set_bit;
				ptr_val[c] = val;

			}
		}
		bin_patterns.push_back(bin_mat.clone());
	}

	cv::Mat k1(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k1 = k1.ptr<ushort>(r);

		for (int c = 0; c < nc; c++)
		{
			std::vector<bool> gray_code_list;

			for (int i = 0; i < bin_patterns.size(); i++)
			{
				uchar val = bin_patterns[i].at<uchar>(r, c);

				if (255 == val)
				{
					gray_code_list.push_back(true);
				}
				else
				{
					gray_code_list.push_back(false);
				}
			}

			ushort k_1 = 0;
			for (int i = 0; i < gray_code_list.size(); i++)
			{
				k_1 += gray_code_list[i] * std::pow(2, gray_code_list.size() - i - 1);

			}
			ptr_k1[c] = k_1;
		}

	}

	k_map = k1.clone();

	uncertain = uncertain_map.clone();

	return true;
	 
}

bool DF_Encode::decodeMinswGrayCode(std::vector<cv::Mat> patterns, std::vector<cv::Mat> threshold_list, cv::Mat& k_map)
{
	if (patterns.size() != threshold_list.size() || patterns.empty())
	{
		return false;
	}

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat bit_value_map(nr, nc, CV_8U, cv::Scalar(0));

	//std::vector<std::vector<bool>> gray_code_list;
	//threshold bin
	std::vector<cv::Mat> bin_patterns;
	for (int i = 0; i < patterns.size(); i++)
	{
		cv::Mat bin_mat(nr, nc, CV_8U, cv::Scalar(0));
		int mv_i = (patterns.size() - i - 1);

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_bin = bin_mat.ptr<uchar>(r);
			uchar* ptr_avg = threshold_list[i].ptr<uchar>(r);
			uchar* ptr_gray = patterns[i].ptr<uchar>(r);
			uchar* ptr_val = bit_value_map.ptr<uchar>(r);
			 

			for (int c = 0; c < nc; c++)
			{
				uchar val = ptr_val[c];
  
				uchar mask = 1 << mv_i; 
				val = val & (~mask);  
				uchar set_bit = 0;
				  
				if (ptr_gray[c] < ptr_avg[c])
				{
					ptr_bin[c] = 0; 
				}
				else
				{
					ptr_bin[c] = 255;
					set_bit = 1 << mv_i;
				}


				val = val | set_bit;
				ptr_val[c] = val;
				 
			}
		}
		bin_patterns.push_back(bin_mat.clone());
	}

	cv::Mat k1(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k1 = k1.ptr<ushort>(r);

		for (int c = 0; c < nc; c++)
		{
			std::vector<bool> gray_code_list;

			for (int i = 0; i < bin_patterns.size(); i++)
			{
				uchar val = bin_patterns[i].at<uchar>(r, c);

				if (255 == val)
				{
					gray_code_list.push_back(true);
				}
				else
				{
					gray_code_list.push_back(false);
				}
			}

			ushort k_1 = 0;
			for (int i = 0; i < gray_code_list.size(); i++)
			{
				k_1 += gray_code_list[i] * std::pow(2, gray_code_list.size() - i - 1);

			}
			ptr_k1[c] = k_1;
		}

	}

	k_map = k1.clone();


	return true;
}

bool DF_Encode::decodeMinswGrayCode(std::vector<cv::Mat> patterns, cv::Mat average_brightness, cv::Mat& k_map)
{
	//bin threshold

	int nr = average_brightness.rows;
	int nc = average_brightness.cols;

	cv::Mat bit_value_map(nr, nc, CV_16U, cv::Scalar(0));

   
	//std::vector<std::vector<bool>> gray_code_list;
	//threshold bin
	std::vector<cv::Mat> bin_patterns;
	for (int i = 0; i < patterns.size(); i++)
	{
		cv::Mat bin_mat(nr, nc, CV_8U, cv::Scalar(0));
		int mv_i = (patterns.size() - i - 1);

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_bin = bin_mat.ptr<uchar>(r);
			uchar* ptr_avg = average_brightness.ptr<uchar>(r);
			uchar* ptr_gray = patterns[i].ptr<uchar>(r);
			ushort* ptr_val = bit_value_map.ptr<ushort>(r);

			for (int c = 0; c < nc; c++)
			{

				ushort val = ptr_val[c];

				ushort mask = 1 << mv_i;
				val = val & (~mask);
				ushort set_bit = 0;

				if (ptr_gray[c] < ptr_avg[c])
				{
					ptr_bin[c] = 0;
				}
				else
				{
					ptr_bin[c] = 255;
					set_bit = 1 << mv_i;
				}


				val = val | set_bit;
				ptr_val[c] = val;

				//if (ptr_gray[c] < ptr_avg[c])
				//{
				//	ptr_bin[c] = 0;
				//}
				//else
				//{
				//	ptr_bin[c] = 255;
				//}
			}
		}
		bin_patterns.push_back(bin_mat.clone());
	}

	cv::Mat k1(nr, nc, CV_16U, cv::Scalar(0)); 


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k1 = k1.ptr<ushort>(r); 

		for (int c = 0; c < nc; c++)
		{
			std::vector<bool> gray_code_list; 

			for (int i = 0; i < bin_patterns.size(); i++)
			{
				uchar val = bin_patterns[i].at<uchar>(r, c);

				if (255 == val)
				{
					gray_code_list.push_back(true);
				}
				else
				{
					gray_code_list.push_back(false);
				}
			}
			  
			ushort k_1 = 0;
			for (int i = 0; i < gray_code_list.size(); i++)
			{
				k_1 += gray_code_list[i] * std::pow(2, gray_code_list.size() - i - 1);
				 
			} 
			ptr_k1[c] = k_1;
		}

	} 

	cv::Mat diff = bit_value_map - k1;

	k_map = k1.clone(); 
	 
	return true;
}

bool DF_Encode::decodeGrayCode(std::vector<cv::Mat> patterns, cv::Mat average_brightness, cv::Mat& k1_map, cv::Mat& k2_map)
{
	//bin threshold

	int nr = average_brightness.rows;
	int nc = average_brightness.cols;

	//std::vector<std::vector<bool>> gray_code_list;
	//threshold bin
	std::vector<cv::Mat> bin_patterns;
	for (int i = 0; i < patterns.size(); i++)
	{
		cv::Mat bin_mat(nr, nc, CV_8U, cv::Scalar(0));

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_bin = bin_mat.ptr<uchar>(r);
			uchar* ptr_avg = average_brightness.ptr<uchar>(r);
			uchar* ptr_gray = patterns[i].ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				if (ptr_gray[c] < ptr_avg[c])
				{
					ptr_bin[c] = 0;
				}
				else
				{
					ptr_bin[c] = 255;
				}
			}
		}
		bin_patterns.push_back(bin_mat.clone());
	}

	cv::Mat k1(nr, nc, CV_16U, cv::Scalar(0));
	cv::Mat k2(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k1 = k1.ptr<ushort>(r);
		ushort* ptr_k2 = k2.ptr<ushort>(r);

		for (int c = 0; c < nc; c++)
		{
			std::vector<bool> gray_code_list;
			std::vector<bool> bin_code_list;

			for (int i = 0; i < bin_patterns.size(); i++)
			{
				uchar val = bin_patterns[i].at<uchar>(r, c);

				if (255 == val)
				{
					gray_code_list.push_back(true);
				}
				else
				{
					gray_code_list.push_back(false);
				}
			}

			grayCodeToBinCode(gray_code_list, bin_code_list);

			ushort k_2 = 0;
			ushort k_1 = 0;
			for (int i = 0; i < bin_code_list.size(); i++)
			{
				k_2 += bin_code_list[i] * std::pow(2, bin_code_list.size() - i - 1);

				if (i < bin_code_list.size() - 1)
				{
					k_1 += bin_code_list[i] * std::pow(2, bin_code_list.size() - i - 2);
				}
			}
			ptr_k2[c] = k_2;
			ptr_k1[c] = k_1;
		}

	}


	k1_map = k1.clone();
	k2_map = k2.clone();


	return true;
}


/**************************************************************************************************************/

bool DF_Encode::mergePatterns(std::vector<std::vector<cv::Mat>> patterns_list, std::vector<cv::Mat>& patterns)
{
	if (patterns_list.empty())
	{
		return false;
	}

	int nr = patterns_list[0][0].rows;
	int nc = patterns_list[0][0].cols;
	int patterns_num = patterns_list[0].size();

	for (int i = 0; i < patterns_num; i++)
	{
		cv::Mat pattern(nr, nc, CV_16U, cv::Scalar(0));

		for (int r = 0; r < nr; r++)
		{

			for (int c = 0; c < nc; c++)
			{
				for (int l_i = 0; l_i < patterns_list.size(); l_i++)
				{
					pattern.at<ushort>(r, c) += patterns_list[l_i][i].at<uchar>(r, c);
				}
			}

		}


		patterns.push_back(pattern.clone());
	}

}

bool DF_Encode::sixStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence)
{
	if (6 != patterns.size())
	{
		return false;
	}

	cv::Mat img0 = patterns[0 + 3];
	cv::Mat img1 = patterns[1 + 3];
	cv::Mat img2 = patterns[2 + 3];
	cv::Mat img3 = patterns[3 - 3];
	cv::Mat img4 = patterns[4 - 3];
	cv::Mat img5 = patterns[5 - 3];


	cv::Mat result(img1.rows, img1.cols, CV_64F, cv::Scalar(-1));
	cv::Mat confidence_map(img1.rows, img1.cols, CV_64F, cv::Scalar(0));

	if (!mask.data)
	{
		mask = cv::Mat(img1.rows, img1.cols, CV_8U, cv::Scalar(255));
	}

	int nl = img1.rows;
	int nc = img1.cols * img1.channels();


	if (img0.isContinuous())
	{
		if (img1.isContinuous())
		{
			if (img2.isContinuous())
			{
				if (img3.isContinuous())
				{
					if (img4.isContinuous())
					{
						if (img5.isContinuous())
						{

							if (mask.isContinuous())
							{
								nc = nc * nl;
								nl = 1;
							}

						}
					}
				}
			}

		}
	}

	if (CV_16U == img0.type())
	{

#pragma omp parallel for
		for (int r = 0; r < nl; r++)
		{

			ushort* ptr0 = img0.ptr<ushort>(r);
			ushort* ptr1 = img1.ptr<ushort>(r);
			ushort* ptr2 = img2.ptr<ushort>(r);
			ushort* ptr3 = img3.ptr<ushort>(r);
			ushort* ptr4 = img4.ptr<ushort>(r);
			ushort* ptr5 = img5.ptr<ushort>(r);
			uchar* ptr_m = mask.ptr<uchar>(r);
			double* ptr_con = confidence_map.ptr<double>(r);

			double* optr = result.ptr<double>(r);
			for (int c = 0; c < nc; c++)
			{
				int exposure_num = 0;
				if (ptr_m[c])
				{
					//double a = ptr4[j] - ptr2[j];
					//double b = ptr1[j] - ptr3[j];
					if (255 == ptr0[c])
					{
						exposure_num++;
					}

					if (255 == ptr1[c])
					{
						exposure_num++;
					}
					if (255 == ptr2[c])
					{
						exposure_num++;
					}
					if (255 == ptr3[c])
					{
						exposure_num++;
					}
					if (255 == ptr4[c])
					{
						exposure_num++;
					}

					if (255 == ptr5[c])
					{
						exposure_num++;
					}


					double b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);


					double a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

					double r = std::sqrt(a * a + b * b);

					//if (r > 255)
					//{
					//	r = 255;
					//}

					ptr_con[c] = r;

					/***********************************************************************/

					if (exposure_num > 3)
					{
						ptr_m[c] = 0;
						ptr_con[c] = 0;
						optr[c] = -1;
					}
					else
					{
						optr[c] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;
	}
	else if (CV_8U == img0.type())
	{

#pragma omp parallel for
		for (int r = 0; r < nl; r++)
		{

			uchar* ptr0 = img0.ptr<uchar>(r);
			uchar* ptr1 = img1.ptr<uchar>(r);
			uchar* ptr2 = img2.ptr<uchar>(r);
			uchar* ptr3 = img3.ptr<uchar>(r);
			uchar* ptr4 = img4.ptr<uchar>(r);
			uchar* ptr5 = img5.ptr<uchar>(r);
			uchar* ptr_m = mask.ptr<uchar>(r);
			double* ptr_con = confidence_map.ptr<double>(r);

			double* optr = result.ptr<double>(r);
			for (int c = 0; c < nc; c++)
			{
				int exposure_num = 0;
				if (ptr_m[c])
				{
					//double a = ptr4[j] - ptr2[j];
					//double b = ptr1[j] - ptr3[j];
					if (255 == ptr0[c])
					{
						exposure_num++;
					}

					if (255 == ptr1[c])
					{
						exposure_num++;
					}
					if (255 == ptr2[c])
					{
						exposure_num++;
					}
					if (255 == ptr3[c])
					{
						exposure_num++;
					}
					if (255 == ptr4[c])
					{
						exposure_num++;
					}

					if (255 == ptr5[c])
					{
						exposure_num++;
					}


					double b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);


					double a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

					double r = std::sqrt(a * a + b * b);

					//if (r > 255)
					//{
					//	r = 255;
					//}

					ptr_con[c] = r;

					/***********************************************************************/

					if (exposure_num > 3)
					{
						ptr_m[c] = 0;
						ptr_con[c] = 0;
						optr[c] = -1;
					}
					else
					{
						optr[c] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;
	}

	return false;
}

bool DF_Encode::fourStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence)
{
	if (4 != patterns.size())
	{
		return false;
	}

	cv::Mat img1 = patterns[0];
	cv::Mat img2 = patterns[1];
	cv::Mat img3 = patterns[2];
	cv::Mat img4 = patterns[3];


	cv::Mat result(img1.rows, img1.cols, CV_64F, cv::Scalar(-1));
	cv::Mat confidence_map(img1.rows, img1.cols, CV_64F, cv::Scalar(0));

	if (!mask.data)
	{
		mask = cv::Mat(img1.rows, img1.cols, CV_8U, cv::Scalar(255));
	}

	int nl = img1.rows;
	int nc = img1.cols * img1.channels();

	if (img1.isContinuous())
	{
		if (img2.isContinuous())
		{
			if (img3.isContinuous())
			{
				if (img4.isContinuous())
				{
					if (mask.isContinuous())
					{
						nc = nc * nl;
						nl = 1;
					}
				}
			}
		}

	}


	if (CV_16U == img1.type())
	{

#pragma omp parallel for
		for (int i = 0; i < nl; i++)
		{
			ushort* ptr1 = img1.ptr<ushort>(i);
			ushort* ptr2 = img2.ptr<ushort>(i);
			ushort* ptr3 = img3.ptr<ushort>(i);
			ushort* ptr4 = img4.ptr<ushort>(i);
			uchar* ptr_m = mask.ptr<uchar>(i);
			double* ptr_con = confidence_map.ptr<double>(i);

			double* optr = result.ptr<double>(i);
			for (int j = 0; j < nc; j++)
			{
				int exposure_num = 0;
				if (ptr_m[j] == 255)
				{
					if (255 == ptr1[j])
					{
						exposure_num++;
					}
					if (255 == ptr2[j])
					{
						exposure_num++;
					}
					if (255 == ptr3[j])
					{
						exposure_num++;
					}
					if (255 == ptr4[j])
					{
						exposure_num++;
					}

					double a = ptr4[j] - ptr2[j];
					double b = ptr1[j] - ptr3[j];

					double r = std::sqrt(a * a + b * b) + 0.5;

					//if(r> 255)
					//{
					//	r = 255;
					//}

					ptr_con[j] = r;

					/***********************************************************************/

					if (exposure_num > 1)
					{
						ptr_m[j] = 0;
						ptr_con[j] = 0;
						optr[j] = -1;
					}
					else
					{
						optr[j] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;

	}
	else if (CV_8U == img1.type())
	{

#pragma omp parallel for
		for (int i = 0; i < nl; i++)
		{
			uchar* ptr1 = img1.ptr<uchar>(i);
			uchar* ptr2 = img2.ptr<uchar>(i);
			uchar* ptr3 = img3.ptr<uchar>(i);
			uchar* ptr4 = img4.ptr<uchar>(i);
			uchar* ptr_m = mask.ptr<uchar>(i);
			double* ptr_con = confidence_map.ptr<double>(i);

			double* optr = result.ptr<double>(i);
			for (int j = 0; j < nc; j++)
			{
				int exposure_num = 0;
				if (ptr_m[j] == 255)
				{
					if (255 == ptr1[j])
					{
						exposure_num++;
					}
					if (255 == ptr2[j])
					{
						exposure_num++;
					}
					if (255 == ptr3[j])
					{
						exposure_num++;
					}
					if (255 == ptr4[j])
					{
						exposure_num++;
					}

					double a = ptr4[j] - ptr2[j];
					double b = ptr1[j] - ptr3[j];

					double r = std::sqrt(a * a + b * b) + 0.5;

					//if(r> 255)
					//{
					//	r = 255;
					//}

					ptr_con[j] = r;

					/***********************************************************************/

					if (exposure_num > 1 || ptr_con[j] < 4)
					{
						ptr_m[j] = 0;
						ptr_con[j] = 0;
						optr[j] = -1;
					}
					else
					{
						optr[j] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;
	}



	return false;
}


bool DF_Encode::unwrapVariableWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, double rate, cv::Mat& h_unwrap, cv::Mat& k_Mat, float threshold, cv::Mat& err_mat)
{

	if (l_unwrap.empty() || h_wrap.empty())
	{
		return false;
	}

	int nr = l_unwrap.rows;
	int nc = l_unwrap.cols;

	if (l_unwrap.isContinuous())
	{
		if (h_wrap.isContinuous())
		{
			if (k_Mat.isContinuous())
			{

				nc = nc * nr;
				nr = 1;
			}
		}

	}

	cv::Mat err_map(l_unwrap.size(), CV_64F, cv::Scalar(0.0));

	for (int r = 0; r < nr; r++)
	{
		double* l_ptr = l_unwrap.ptr<double>(r);
		double* h_ptr = h_wrap.ptr<double>(r);
		uchar* k_ptr = k_Mat.ptr<uchar>(r);
		double* h_unwrap_ptr = h_unwrap.ptr<double>(r);

		double* ptr_err = err_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{

			//double temp = 0.5 + l_ptr[j] / (1 * CV_PI) - h_ptr[j] / (rate * CV_PI); 

			double temp = 0.5 + (rate * l_ptr[c] - h_ptr[c]) / (2 * CV_PI);
			int k = temp;
			h_unwrap_ptr[c] = 2 * CV_PI * k + h_ptr[c];

			ptr_err[c] = fabs(h_unwrap_ptr[c] - rate * l_ptr[c]);

			k_ptr[c] = k;

			if (ptr_err[c] > threshold)
			{
				h_unwrap_ptr[c] = -10;

			}

			//int k = temp; 
			//k_ptr[j] = k; 
			//h_unwrap_ptr[j] = 2 * CV_PI * k + h_ptr[j];


		}
	}

	err_mat = err_map.clone();

	return true;
}

void DF_Encode::unwarpDualWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, cv::Mat& h_unwrap, cv::Mat& k_Mat)
{


	int nr = l_unwrap.rows;
	int nc = l_unwrap.cols;

	if (l_unwrap.isContinuous())
	{
		if (h_wrap.isContinuous())
		{
			if (k_Mat.isContinuous())
			{

				nc = nc * nr;
				nr = 1;
			}
		}

	}


	for (int i = 0; i < nr; i++)
	{
		double* l_ptr = l_unwrap.ptr<double>(i);
		double* h_ptr = h_wrap.ptr<double>(i);
		uchar* k_ptr = k_Mat.ptr<uchar>(i);
		double* h_unwrap_ptr = h_unwrap.ptr<double>(i);
		for (int j = 0; j < nc; j++)
		{

			double temp = 0.5 + l_ptr[j] / (1 * CV_PI) - h_ptr[j] / (2 * CV_PI);

			int k = temp;


			k_ptr[j] = k;

			h_unwrap_ptr[j] = 2 * CV_PI * k + h_ptr[j];




		}
	}


}

bool DF_Encode::unwrapHalfWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, cv::Mat& unwrap_img, cv::Mat& mask)
{
	if (wrap_img_list.empty())
	{
		return false;
	}



	bool unwrap_filter = false;


	if (mask.data)
	{
		unwrap_filter = true;
	}

	std::vector<cv::Mat> unwrap_img_list;
	unwrap_img_list.push_back(wrap_img_list[0]);

	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	for (int w_i = 0; w_i < wrap_img_list.size() - 1; w_i++)
	{
		cv::Mat img_1 = unwrap_img_list[w_i];
		cv::Mat img_2 = wrap_img_list[w_i + 1];

		std::cout << "w_i: " << w_i;

		cv::Mat k_mat(nr, nc, CV_8U, cv::Scalar(0));
		cv::Mat unwrap_mat(nr, nc, CV_64F, cv::Scalar(0));

		unwarpDualWavelength(img_1, img_2, unwrap_mat, k_mat);
		unwrap_img_list.push_back(unwrap_mat);

	}


	float period_num = std::pow(2, unwrap_img_list.size() - 1);



	//unwrap_img = unwrap_img_list[unwrap_img_list.size() - 1].clone()/ period_num;

	unwrap_img = unwrap_img_list[unwrap_img_list.size() - 1].clone();


	return true;
}


bool DF_Encode::unwrapVariableWavelengthPatterns(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask)
{
	if (wrap_img_list.empty())
	{
		return false;
	}
	if (wrap_img_list.size() != rate_list.size() + 1)
	{
		return false;
	}

	std::vector<float> threshold_list;

	for (int i = 0; i < rate_list.size(); i++)
	{
		threshold_list.push_back(CV_PI);
	}


	if (threshold_list.size() >= 3)
	{
		threshold_list[0] = CV_PI;
		threshold_list[1] = CV_PI;
		threshold_list[2] = 1.5;
	}


	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	bool unwrap_filter = false;

	if (mask.data)
	{
		unwrap_filter = true;
	}

	cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat err_map_l(nr, nc, CV_64F, cv::Scalar(0));
	cv::Mat err_map_h(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat unwrap_map = wrap_img_list[0];

	cv::Mat k_mat(nr, nc, CV_8U, cv::Scalar(0));

	for (int g_i = 1; g_i < wrap_img_list.size(); g_i++)
	{
		cv::Mat wrap_map = wrap_img_list[g_i];
		cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));
		cv::Mat err_map;

		unwrapVariableWavelength(unwrap_map, wrap_map, rate_list[g_i - 1], h_unwrap_map, k_mat, threshold_list[g_i - 1], err_map);

		unwrap_map = h_unwrap_map.clone();
	}

	unwrap_img = unwrap_map.clone();

	return true;
}

bool DF_Encode::unwrapVariableWavelengthPatternsBaseConfidence(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask)
{
	if (wrap_img_list.empty())
	{
		return false;
	}
	if (wrap_img_list.size() != rate_list.size() + 1)
	{
		return false;
	}

	std::vector<float> threshold_list;

	for (int i = 0; i < rate_list.size(); i++)
	{
		threshold_list.push_back(CV_PI);
	}


	if (threshold_list.size() >= 3)
	{
		threshold_list[0] = CV_PI;
		threshold_list[1] = CV_PI;
		threshold_list[2] = 1.5;
	}


	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	bool unwrap_filter = false;

	if (mask.data)
	{
		unwrap_filter = true;
	}

	cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat err_map_l(nr, nc, CV_64F, cv::Scalar(0));
	cv::Mat err_map_h(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat unwrap_map = wrap_img_list[0];

	cv::Mat k_mat(nr, nc, CV_8U, cv::Scalar(0));

	for (int g_i = 1; g_i < wrap_img_list.size(); g_i++)
	{
		cv::Mat wrap_map = wrap_img_list[g_i];
		cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));
		cv::Mat err_map;

		unwrapVariableWavelength(unwrap_map, wrap_map, rate_list[g_i - 1], h_unwrap_map, k_mat, threshold_list[g_i - 1], err_map);

		const double fisher_rates[] = { -6.61284856e-06, 4.52035763e-06, -1.16182132e-05 };//[-6.61284856e-06  4.52035763e-06 -1.16182132e-05 -2.89004663e-05]
		double fisher_temp = fisher_rates[g_i - 1];
		for (int r = 0; r < mask.rows; r += 1)
		{
			double* mask_ptr = mask.ptr<double>(r);
			double* err_ptr = err_map.ptr<double>(r);
			for (int c = 0; c < mask.cols; c += 1)
			{
				mask_ptr[c] += err_ptr[c] * fisher_temp;
			}
		}

		unwrap_map = h_unwrap_map.clone();
	}

	unwrap_img = unwrap_map.clone();

	cv::Mat neighborhoodCharacteristicsR(unwrap_map.size(), CV_64F, cv::Scalar(0));
	// 注意避坑：邻域的计算当中应当避免出现的问题是
	for (int r = 0; r < neighborhoodCharacteristicsR.rows; r += 1)
	{
		double* neighborhoodR_Ptr = neighborhoodCharacteristicsR.ptr<double>(r);
		double* data_ptr = unwrap_map.ptr<double>(r);
		double* mask_ptr = mask.ptr<double>(r);

		int numR = 0, numC = 0;

		for (int c = 1; c < neighborhoodCharacteristicsR.cols - 1; c += 1)
		{
			// 在此处需要循环找到非-10的值
			while (data_ptr[c] == -10 && c < neighborhoodCharacteristicsR.cols - 1)
			{
				c += 1;
				mask_ptr[c] = -10;
			}
			numC = c;
			while (data_ptr[c + 1] == -10 && c < neighborhoodCharacteristicsR.cols - 1)
			{
				c += 1;
				mask_ptr[c] = -10;
			}
			numR = c + 1;
			neighborhoodR_Ptr[c] = data_ptr[numR] - data_ptr[numC];
		}
	}
	unwrap_img = unwrap_map.clone();

	// 根据右减左来写代码

	for (int r = 0; r < neighborhoodCharacteristicsR.rows; r += 1)
	{
		double* neighborPtr = neighborhoodCharacteristicsR.ptr<double>(r);
		for (int c = 0; c < neighborhoodCharacteristicsR.cols; c += 1)
		{
			if (neighborPtr[c] < -1)
			{
				neighborPtr[c] = -1;
			}
			else if (neighborPtr[c] > 1)
			{
				neighborPtr[c] = 1;
			}
			neighborPtr[c] = neighborPtr[c] * (-0.5) + 0.5;
		}
	}

	cv::Mat kernal = (cv::Mat_<double>(1, 7) << 1, 1, 1, 1, 1, 1, 1);
	cv::dilate(neighborhoodCharacteristicsR, neighborhoodCharacteristicsR, kernal, cv::Point(0, 0), 1);

	for (int r = 0; r < neighborhoodCharacteristicsR.rows; r += 1)
	{
		double* neighborPtr = neighborhoodCharacteristicsR.ptr<double>(r);
		double* maskPtr = mask.ptr<double>(r);
		for (int c = 0; c < neighborhoodCharacteristicsR.cols; c += 1)
		{
			maskPtr[c] += neighborPtr[c] * (-2.89004663e-05);

		}
	}

	return true;
}


bool DF_Encode::unwrapVariableWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask)
{

	if (wrap_img_list.empty())
	{
		return false;
	}
	if (3 != wrap_img_list.size())
	{
		return false;
	}

	if (2 != rate_list.size())
	{
		return false;
	}

	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	bool unwrap_filter = false;

	if (mask.data)
	{
		unwrap_filter = true;
	}

	cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat err_map_l(nr, nc, CV_64F, cv::Scalar(0));
	cv::Mat err_map_h(nr, nc, CV_64F, cv::Scalar(0));

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		double* ptr_0 = wrap_img_list[0].ptr<double>(r);
		double* ptr_1 = wrap_img_list[1].ptr<double>(r);
		double* ptr_2 = wrap_img_list[2].ptr<double>(r);

		double* ptr_err_l = err_map_l.ptr<double>(r);
		double* ptr_err_h = err_map_h.ptr<double>(r);

		uchar* ptr_mask = mask.ptr<uchar>(r);

		double* ptr_h = h_unwrap_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{

			double temp = 0.5 + (rate_list[0] * ptr_0[c] - ptr_1[c]) / (CV_PI);
			int k = temp;
			ptr_h[c] = CV_PI * k + ptr_1[c];

			if (unwrap_filter)
			{
				double error = fabs(ptr_h[c] - ptr_0[c] * rate_list[0]);
				ptr_err_l[c] = error * 1;
				//backup 0.5
				if (error > 1.0)
				{
					ptr_h[c] = 0;
					ptr_mask[c] = 0;
				}
			}




			/******************************************************************/
			temp = 0.5 + (rate_list[1] * ptr_h[c] - ptr_2[c]) / (CV_PI);
			k = temp;

			double old_ptr_h = ptr_h[c];
			ptr_h[c] = CV_PI * k + ptr_2[c];

			if (unwrap_filter)
			{
				double error = fabs(ptr_h[c] - old_ptr_h * rate_list[1]);
				ptr_err_h[c] = error * 1;
				//backup 0.2
				if (error > 0.4)
				{
					ptr_h[c] = 0;
					ptr_mask[c] = 0;
				}
			}


			/********************************************************************************/
		}

	}

	unwrap_img = h_unwrap_map.clone();

	//unwrap_img = unwrap_img / 32;

	return true;
}



bool DF_Encode::computePhaseBaseSixStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence)
{
	std::vector<cv::Mat> wrap_img_list;
	std::vector<cv::Mat> confidence_map_list;
	std::vector<int> number_list;



#pragma omp parallel for
	for (int i = 0; i < patterns.size() - 1; i += 6)
	{
		cv::Mat wrap_img;
		cv::Mat confidence;

		std::vector<cv::Mat> phase_list(patterns.begin() + i, patterns.begin() + i + 6);
		sixStepPhaseShift(phase_list, wrap_img, mask_img, confidence);


#pragma omp critical
		{
			number_list.push_back(i / 6);
			wrap_img_list.push_back(wrap_img);
			confidence_map_list.push_back(confidence);
		}


	}


	std::vector<cv::Mat> sort_img_list;
	sort_img_list.resize(wrap_img_list.size());

	std::vector<cv::Mat> sort_confidencce_list;
	sort_confidencce_list.resize(confidence_map_list.size());

	for (int i = 0; i < wrap_img_list.size(); i++)
	{

		sort_img_list[number_list[i]] = wrap_img_list[i];
		sort_confidencce_list[number_list[i]] = confidence_map_list[i];
	}

	wrap_maps = sort_img_list;

	cv::Mat confid_map = sort_confidencce_list[0].clone();

	int nr = sort_confidencce_list[0].rows;
	int nc = sort_confidencce_list[0].cols;

	//for (int r = 0; r < nr; r++)
	//{

	//	double* ptr_0 = sort_confidencce_list[0].ptr<double>(r);
	//	double* ptr_1 = sort_confidencce_list[1].ptr<double>(r);
	//	double* ptr_2 = sort_confidencce_list[2].ptr<double>(r);

	//	double* ptr_confid = confid_map.ptr<double>(r);

	//	for (int c = 0; c < nc; c++)
	//	{

	//		double max_v = 0;
	//		if (ptr_0[c] > ptr_1[c])
	//		{
	//			max_v = ptr_0[c];
	//		}
	//		else
	//		{
	//			max_v = ptr_1[c];
	//		}
	//		if (max_v < ptr_2[c])
	//		{
	//			max_v = ptr_2[c];
	//		}
	//		ptr_confid[c] = max_v;

	//	}
	//}

	confidence = confid_map.clone();


	return true;
}

bool DF_Encode::computePhaseBaseFourStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence)
{

	std::vector<cv::Mat> wrap_img_list;
	std::vector<cv::Mat> confidence_map_list;
	std::vector<int> number_list;



#pragma omp parallel for
	for (int i = 0; i < patterns.size() - 1; i += 4)
	{
		cv::Mat wrap_img;
		cv::Mat confidence;

		std::vector<cv::Mat> phase_list(patterns.begin() + i, patterns.begin() + i + 4);
		fourStepPhaseShift(phase_list, wrap_img, mask_img, confidence);


#pragma omp critical
		{
			number_list.push_back(i / 4);
			wrap_img_list.push_back(wrap_img);
			confidence_map_list.push_back(confidence);
		}


	}


	std::vector<cv::Mat> sort_img_list;
	sort_img_list.resize(wrap_img_list.size());

	std::vector<cv::Mat> sort_confidencce_list;
	sort_confidencce_list.resize(confidence_map_list.size());

	for (int i = 0; i < wrap_img_list.size(); i++)
	{

		sort_img_list[number_list[i]] = wrap_img_list[i];
		sort_confidencce_list[number_list[i]] = confidence_map_list[i];
	}

	wrap_maps = sort_img_list;

	cv::Mat confid_map = sort_confidencce_list[0].clone();

	int nr = sort_confidencce_list[0].rows;
	int nc = sort_confidencce_list[0].cols;

	for (int r = 0; r < nr; r++)
	{

		uchar* ptr_0 = sort_confidencce_list[0].ptr<uchar>(r);
		uchar* ptr_1 = sort_confidencce_list[1].ptr<uchar>(r);
		uchar* ptr_2 = sort_confidencce_list[2].ptr<uchar>(r);

		uchar* ptr_confid = confid_map.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			uchar min_v = 255;
			if (ptr_0[c] < ptr_1[c])
			{
				min_v = ptr_0[c];
			}
			else
			{
				min_v = ptr_1[c];
			}
			if (min_v > ptr_2[c])
			{
				min_v = ptr_2[c];
			}
			ptr_confid[c] = min_v;

			//uchar max_v = 0;
			//if (ptr_0[c] > ptr_1[c])
			//{
			//	max_v = ptr_0[c];
			//}
			//else
			//{
			//	max_v = ptr_1[c];
			//}
			//if (max_v < ptr_2[c])
			//{
			//	max_v = ptr_2[c];
			//}
			//ptr_confid[c] = max_v;

		}
	}

	confidence = confid_map.clone();


	return true;
}


bool DF_Encode::maskMap(cv::Mat mask, cv::Mat& map)
{
	if (!mask.data)
	{
		return false;
	}

	if (!map.data)
	{
		return false;
	}


	if (CV_64FC3 == map.type())
	{
		for (int r = 0; r < map.rows; r++)
		{

			cv::Vec3d* ptr_map = map.ptr<cv::Vec3d>(r);
			uchar* ptr_mask = mask.ptr<uchar>(r);

			for (int c = 0; c < map.cols; c++)
			{
				if (0 == ptr_mask[c])
				{
					ptr_map[c][0] = 0;
					ptr_map[c][1] = 0;
					ptr_map[c][2] = 0;
				}
			}

		}
	}
	else if (CV_64FC1 == map.type())
	{
		for (int r = 0; r < map.rows; r++)
		{
			double* ptr_map = map.ptr<double>(r);
			uchar* ptr_mask = mask.ptr<uchar>(r);

			for (int c = 0; c < map.cols; c++)
			{
				if (0 == ptr_mask[c])
				{
					ptr_map[c] = 0;
				}
			}

		}
	}



	return true;
}

bool DF_Encode::selectMaskBaseConfidence(cv::Mat confidence, int threshold, cv::Mat& mask)
{
	if (!confidence.data)
	{
		return false;
	}

	cv::Mat confidence_map = confidence.clone();
	if (confidence_map.type() != CV_64FC1)
	{
		confidence_map.convertTo(confidence_map, CV_64FC1);
	}

	int nr = confidence.rows;
	int nc = confidence.cols;

	//mask = cv::Mat(nr, nc, CV_8U, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		double* ptr_c = confidence_map.ptr<double>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			if (ptr_c[c] < threshold)
			{
				ptr_m[c] = 0;
			} 

		}
	}

	return true;

}
