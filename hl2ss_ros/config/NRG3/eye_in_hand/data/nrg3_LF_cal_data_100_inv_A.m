%% Results

%% Raw Data
function [q_headset2world, q_target2cam, t_headset2world, t_target2cam] = nrg3_LF_cal_data_100_inv_A()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HoloLens 2 'NRG3' LeftFront Eye-In-Hand Calibration Data (Inverted)
% 
% About:
%   Function to grab raw data to perform a eye-in-hand calibration
%   on the Left Front VLC camera of the NRG3 HoloLens 2. It was obtained 
%   using the Left Front VLC Camera with an AprilTag and using the headset 
%   pose obtained w.r.t. world from a Unity application. 
% 
% Input:
%   None
% 
% Output:
%   Transforms for eye-in-hand calibration
% 
% Format:
%   ROS Coord Conv: RH, x=forward,y=-right, z=up
%   Position      : [x, y, z]
%   Quaternion    : [x, y, z, w]
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Position data from 'holoLens/base_link' to 'world' (t_headset2world)
% [x, y, z]
t_headset2world=[
   -0.6681   -0.4679    0.4804
   -0.6182   -0.7236    0.4805
   -0.6829   -0.8294    0.4743
   -0.6912   -0.4723    0.4770
   -0.7446   -0.1602    0.4749
   -0.7734    0.0937    0.4714
   -0.7682    0.3222    0.4696
   -0.7413    0.4683    0.4718
   -0.8248    0.2137    0.4669
   -0.8487   -0.0966    0.4637
   -0.8400   -0.3473    0.4628
   -0.7442   -0.6819    0.4663
   -0.5280   -0.9533    0.4795
   -0.5286   -0.7620    0.4851
   -0.5303   -0.5670    0.4865
   -0.6224   -0.2904    0.4812
   -0.6570   -0.0135    0.4790
   -0.6625    0.1545    0.4781
   -0.8112    0.1606    0.4642
   -0.9013   -0.1185    0.4611
   -0.7310    0.0992    0.4755
   -0.6554   -0.1692    0.4775
   -0.5527   -0.3191    0.4846
   -0.4645   -0.5852    0.4886
   -0.3903   -0.7861    0.4928
   -0.3056   -0.9253    0.4987
   -0.2585   -0.7481    0.5051
   -0.5252   -0.3913    0.4844
   -0.5184   -0.1734    0.4852
   -0.6456    0.0152    0.4753
   -0.6177    0.2017    0.4772
   -0.8342    0.0303    0.4581
   -0.8435   -0.2913    0.4561
   -0.8022   -0.5874    0.4571
   -0.5983   -0.6345    0.4743
   -0.6291    0.0135    0.4746
   -0.8068    0.1271    0.4579
   -0.6798    0.3327    0.4691
   -0.8069    0.0393    0.4577
   -0.8583   -0.2570    0.4536
   -0.9390   -0.3584    0.4454
   -0.7739   -0.6998    0.4567
   -0.6579   -0.6450    0.4671
   -0.4735   -0.5684    0.4820
   -0.5197   -0.3444    0.4800
   -0.5740   -0.0096    0.4747
   -0.1199   -1.0004    0.5037
   -0.1271   -1.1722    0.5014
   -0.2632   -1.1882    0.4927
   -0.8426   -0.4519    0.4479
   -0.8339   -0.4554    0.3244
   -0.8047   -0.2089    0.3244
   -0.8252    0.0309    0.3179
   -0.8812    0.1748    0.3078
   -0.9556    0.2489    0.2988
   -0.9625   -0.3785    0.3061
   -0.8177   -0.7070    0.3199
   -0.6030   -0.9701    0.3366
   -0.4288   -1.1319    0.3521
   -0.3676   -1.0215    0.3572
   -0.3439   -0.8773    0.3571
   -0.4183   -0.7067    0.3505
   -0.4183   -0.5331    0.3478
   -0.4819   -0.2713    0.3414
   -0.5205   -0.0025    0.3324
   -0.5831    0.0757    0.3267
   -0.5391    0.3334    0.3236
   -0.5236    0.4688    0.3249
   -0.4026    0.4755    0.3336
   -0.3320    0.1941    0.3427
   -0.3077   -0.1027    0.3469
   -0.4207   -0.5781    0.3438
   -0.3621   -0.8229    0.3495
   -0.2683   -1.0093    0.3584
   -0.0433   -1.1646    0.3733
    0.2072   -1.1885    0.8331
   -0.1479   -1.0373    0.9239
   -0.4776   -0.6884    1.0037
   -0.5686   -0.4126    1.0241
   -0.6143   -0.1128    1.0340
   -0.6019    0.1335    1.0271
   -0.5217    0.3330    1.0045
   -0.3906    0.4323    0.9733
   -0.2338    0.4504    0.9433
   -0.3023    0.0889    0.9576
   -0.2448   -0.2321    0.9445
   -0.1696   -0.5337    0.9263
   -0.0593   -0.8171    0.8997
    0.1028   -1.0210    0.8595
    0.2739   -1.0411    0.8157
    0.3739   -1.0147    0.7921
    0.1746   -0.8220    0.8433
   -0.1690   -0.1543    0.9237
   -0.2164    0.0376    0.9348
   -0.2808    0.1544    0.9487
   -0.4223    0.2758    0.9816
   -0.5497    0.3145    1.0129
   -0.6057    0.3113    1.0265
   -0.4811    0.2785    1.0000
   -0.5008    0.0899    1.0055
];

% Rotation data from 'holoLens/base_link' to 'world' (q_headset2world)
% [x, y, z, w]
q_headset2world=[
    0.0009   -0.0377   -0.0184    0.9991
   -0.0023   -0.0378    0.0863    0.9955
   -0.0028   -0.0383    0.0949    0.9947
    0.0009   -0.0370   -0.0186    0.9991
    0.0018   -0.0369   -0.0815    0.9960
    0.0045   -0.0374   -0.1490    0.9881
    0.0071   -0.0386   -0.1960    0.9798
    0.0094   -0.0381   -0.2343    0.9714
    0.0089   -0.0368   -0.1749    0.9839
    0.0027   -0.0375   -0.0892    0.9953
    0.0015   -0.0379   -0.0368    0.9986
   -0.0018   -0.0384    0.0735    0.9966
   -0.0053   -0.0381    0.1921    0.9806
   -0.0028   -0.0372    0.0995    0.9943
   -0.0007   -0.0373    0.0328    0.9988
    0.0021   -0.0371   -0.0739    0.9966
    0.0038   -0.0369   -0.1314    0.9906
    0.0061   -0.0373   -0.1698    0.9848
    0.0063   -0.0383   -0.1840    0.9822
    0.0007   -0.0380   -0.0654    0.9971
    0.0040   -0.0371   -0.1387    0.9896
    0.0022   -0.0368   -0.0601    0.9975
    0.0011   -0.0368   -0.0273    0.9989
   -0.0005   -0.0374    0.0342    0.9987
   -0.0026   -0.0376    0.0970    0.9946
   -0.0048   -0.0372    0.1489    0.9881
   -0.0035   -0.0373    0.0964    0.9946
    0.0032   -0.0376   -0.0931    0.9949
    0.0020   -0.0370   -0.0835    0.9958
    0.0039   -0.0377   -0.1262    0.9913
    0.0061   -0.0376   -0.1540    0.9873
    0.0037   -0.0387   -0.1207    0.9919
    0.0003   -0.0384   -0.0161    0.9991
   -0.0020   -0.0387    0.0689    0.9969
   -0.0015   -0.0375    0.0697    0.9969
    0.0055   -0.0373   -0.1590    0.9866
    0.0052   -0.0388   -0.1605    0.9863
    0.0097   -0.0378   -0.2293    0.9726
    0.0048   -0.0385   -0.1297    0.9908
    0.0017   -0.0384   -0.0541    0.9978
    0.0013   -0.0392   -0.0404    0.9984
   -0.0039   -0.0385    0.1192    0.9921
   -0.0021   -0.0378    0.0825    0.9959
   -0.0024   -0.0377    0.0814    0.9960
    0.0005   -0.0378   -0.0221    0.9990
    0.0052   -0.0380   -0.1302    0.9907
   -0.0080   -0.0376    0.2391    0.9702
   -0.0068   -0.0385    0.2229    0.9741
   -0.0097   -0.0388    0.2427    0.9693
    0.0001   -0.0393   -0.0037    0.9992
   -0.0084   -0.0333   -0.0058    0.9994
   -0.0065   -0.0327   -0.0753    0.9966
   -0.0050   -0.0342   -0.1354    0.9902
   -0.0029   -0.0358   -0.1712    0.9846
   -0.0015   -0.0366   -0.2011    0.9789
   -0.0072   -0.0366   -0.0231    0.9990
   -0.0093   -0.0357    0.0701    0.9969
   -0.0112   -0.0346    0.1590    0.9866
   -0.0139   -0.0338    0.2147    0.9760
   -0.0155   -0.0332    0.2287    0.9728
   -0.0140   -0.0335    0.1930    0.9805
   -0.0120   -0.0344    0.1194    0.9922
   -0.0098   -0.0349    0.0633    0.9973
   -0.0072   -0.0354   -0.0465    0.9983
   -0.0008   -0.0360   -0.1704    0.9847
    0.0001   -0.0364   -0.2029    0.9785
    0.0032   -0.0387   -0.2525    0.9668
    0.0039   -0.0375   -0.2671    0.9629
    0.0049   -0.0361   -0.2695    0.9623
    0.0056   -0.0363   -0.2240    0.9739
   -0.0019   -0.0365   -0.1212    0.9920
   -0.0087   -0.0360    0.0358    0.9987
   -0.0109   -0.0350    0.1181    0.9923
   -0.0136   -0.0334    0.1941    0.9803
   -0.0144   -0.0332    0.2393    0.9703
    0.0256    0.1203    0.2230    0.9670
    0.0155    0.1219    0.1547    0.9803
    0.0020    0.1226    0.0432    0.9915
   -0.0073    0.1218   -0.0237    0.9922
   -0.0183    0.1211   -0.1031    0.9871
   -0.0256    0.1190   -0.1707    0.9778
   -0.0316    0.1171   -0.2307    0.9654
   -0.0345    0.1168   -0.2627    0.9572
   -0.0391    0.1159   -0.3004    0.9459
   -0.0254    0.1210   -0.1942    0.9731
   -0.0111    0.1231   -0.0653    0.9902
    0.0004    0.1236    0.0272    0.9920
    0.0112    0.1229    0.1096    0.9863
    0.0199    0.1224    0.1789    0.9760
    0.0248    0.1218    0.2146    0.9688
    0.0247    0.1218    0.2207    0.9674
    0.0076    0.1237    0.0922    0.9880
   -0.0296    0.1195   -0.2146    0.9689
   -0.0303    0.1197   -0.2235    0.9668
   -0.0275    0.1192   -0.1955    0.9730
   -0.0329    0.1167   -0.2375    0.9638
   -0.0345    0.1163   -0.2492    0.9608
   -0.0337    0.1165   -0.2454    0.9618
   -0.0329    0.1189   -0.2539    0.9593
   -0.0253    0.1209   -0.1802    0.9758
];

% Position data from 'april_tag' to 'hololens/rm_vlc_leftfront' (t_cam2target)
% [x, y, z]
t_target2cam=[
   -0.6558    0.1536    0.1285
   -0.5592    0.2496    0.1322
   -0.4657    0.3203    0.1298
   -0.6324    0.1520    0.1286
   -0.6146   -0.0630    0.1206
   -0.5610   -0.2144    0.1339
   -0.5035   -0.3556    0.1362
   -0.4679   -0.4368    0.1387
   -0.4832   -0.2765    0.1361
   -0.5080   -0.1071    0.1271
   -0.5001    0.0553    0.1310
   -0.4613    0.1953    0.1296
   -0.4404    0.2974    0.1274
   -0.6256    0.2710    0.1213
   -0.7336    0.1737    0.1204
   -0.7448    0.0304    0.1235
   -0.6895   -0.1792    0.1371
   -0.6531   -0.2862    0.1362
   -0.5212   -0.2225    0.1304
   -0.4395   -0.1271    0.1321
   -0.5943   -0.2495    0.1353
   -0.6860   -0.1214    0.1275
   -0.7760   -0.0216    0.1238
   -0.7957    0.1837    0.1249
   -0.7558    0.3201    0.1187
   -0.7224    0.4089    0.1257
   -0.8969    0.3055    0.1182
   -0.8737    0.1247    0.1176
   -0.8352   -0.1104    0.1315
   -0.6976   -0.2248    0.1291
   -0.6789   -0.3794    0.1366
   -0.5108   -0.2062    0.1317
   -0.4831   -0.0673    0.1297
   -0.4280    0.0797    0.1325
   -0.6205    0.1555    0.1239
   -0.7292   -0.1973    0.1287
   -0.5275   -0.2488    0.1351
   -0.5847   -0.3780    0.1355
   -0.5394   -0.2161    0.1314
   -0.4948   -0.0313    0.1299
   -0.4094    0.0456    0.1318
   -0.3824    0.0898    0.1329
   -0.5435    0.1335    0.1270
   -0.7391    0.0887    0.1245
   -0.8065   -0.0228    0.1326
   -0.7776   -0.2261    0.1302
   -0.7121    0.4068    0.1256
   -0.6573    0.5810    0.1195
   -0.4939    0.4947    0.1244
   -0.4745    0.0538    0.1285
   -0.4700    0.0621    0.2744
   -0.5460   -0.0680    0.2758
   -0.5188   -0.2026    0.2759
   -0.4410   -0.2695    0.2768
   -0.3579   -0.2551    0.2765
   -0.3625    0.0121    0.2737
   -0.3830    0.1635    0.2717
   -0.4050    0.2987    0.2696
   -0.4108    0.4067    0.2678
   -0.4920    0.3063    0.2699
   -0.6332    0.2443    0.2650
   -0.7098    0.1618    0.2712
   -0.8082    0.0579    0.2727
   -0.8529   -0.0950    0.2679
   -0.8408   -0.2244    0.2641
   -0.7705   -0.2415    0.2652
   -0.7107   -0.4275    0.2782
   -0.6527   -0.5424    0.2765
   -0.7512   -0.6086    0.2790
   -0.9394   -0.4439    0.2831
   -1.0496   -0.2285    0.2564
   -0.8307    0.1188    0.2686
   -0.7398    0.2692    0.2684
   -0.6477    0.3764    0.2653
   -0.6914    0.5475    0.2622
   -0.7394    0.5812   -0.2162
   -0.5990    0.3978   -0.2124
   -0.5195    0.1641   -0.2069
   -0.5174    0.0030   -0.2014
   -0.5051   -0.1525   -0.1981
   -0.4939   -0.2805   -0.1934
   -0.5148   -0.3970   -0.1890
   -0.5859   -0.4937   -0.1905
   -0.7061   -0.5129   -0.1904
   -0.8029   -0.3050   -0.1861
   -0.8624   -0.1453   -0.1991
   -0.8628    0.0740   -0.2027
   -0.8216    0.2692   -0.2122
   -0.7965    0.4354   -0.2157
   -0.8759    0.4826   -0.2245
   -0.9656    0.5032   -0.2206
   -1.0813    0.3276   -0.2189
   -1.0369   -0.1187   -0.1910
   -0.9148   -0.2646   -0.1894
   -0.8020   -0.3754   -0.1914
   -0.6271   -0.3784   -0.1951
   -0.5012   -0.3307   -0.1945
   -0.4496   -0.3104   -0.1953
   -0.5764   -0.3282   -0.1947
   -0.6081   -0.2571   -0.1942
];

% Rotation data from 'april_tag' to 'hololens/rm_vlc_leftfront' (q_cam2target)
% [x, y, z, w]
q_target2cam=[
   -0.0131    0.1074    0.0136    0.9940
   -0.0015    0.1104   -0.0893    0.9899
   -0.0015    0.1103   -0.0960    0.9892
   -0.0136    0.1069    0.0115    0.9941
   -0.0187    0.0974    0.0739    0.9923
   -0.0244    0.1071    0.1510    0.9824
   -0.0304    0.1085    0.1974    0.9738
   -0.0354    0.1091    0.2359    0.9650
   -0.0308    0.1073    0.1763    0.9780
   -0.0193    0.1007    0.0832    0.9912
   -0.0148    0.1077    0.0284    0.9937
   -0.0042    0.1086   -0.0779    0.9910
    0.0073    0.1054   -0.1906    0.9760
   -0.0020    0.1015   -0.1003    0.9898
   -0.0078    0.1015   -0.0384    0.9941
   -0.0179    0.1022    0.0670    0.9923
   -0.0228    0.1097    0.1362    0.9843
   -0.0271    0.1087    0.1706    0.9790
   -0.0302    0.1045    0.1851    0.9767
   -0.0150    0.1062    0.0683    0.9919
   -0.0250    0.1073    0.1396    0.9841
   -0.0167    0.1026    0.0619    0.9927
   -0.0141    0.1016    0.0305    0.9943
   -0.0083    0.1048   -0.0378    0.9937
   -0.0026    0.1023   -0.0998    0.9897
    0.0051    0.1050   -0.1483    0.9833
   -0.0007    0.1019   -0.0944    0.9903
   -0.0210    0.1016    0.0862    0.9909
   -0.0184    0.1062    0.0824    0.9908
   -0.0232    0.1042    0.1264    0.9862
   -0.0271    0.1077    0.1565    0.9814
   -0.0222    0.1061    0.1217    0.9866
   -0.0116    0.1043    0.0160    0.9943
   -0.0040    0.1099   -0.0701    0.9915
   -0.0047    0.1022   -0.0699    0.9923
   -0.0270    0.1039    0.1596    0.9813
   -0.0259    0.1092    0.1603    0.9807
   -0.0349    0.1068    0.2293    0.9668
   -0.0233    0.1056    0.1301    0.9856
   -0.0157    0.1054    0.0451    0.9933
   -0.0144    0.1101    0.0317    0.9933
    0.0020    0.1096   -0.1188    0.9869
   -0.0028    0.1045   -0.0887    0.9906
   -0.0021    0.1029   -0.0831    0.9912
   -0.0128    0.1082    0.0229    0.9938
   -0.0251    0.1053    0.1259    0.9861
    0.0138    0.1036   -0.2359    0.9661
    0.0111    0.1036   -0.2218    0.9695
    0.0162    0.1052   -0.2403    0.9648
   -0.0104    0.1063    0.0001    0.9943
   -0.0025    0.1060    0.0039    0.9944
   -0.0083    0.1041    0.0722    0.9919
   -0.0133    0.1043    0.1336    0.9854
   -0.0178    0.1059    0.1706    0.9795
   -0.0210    0.1062    0.1999    0.9738
   -0.0045    0.1079    0.0202    0.9939
    0.0049    0.1073   -0.0693    0.9918
    0.0129    0.1042   -0.1562    0.9821
    0.0198    0.1022   -0.2122    0.9717
    0.0213    0.1016   -0.2249    0.9688
    0.0182    0.0970   -0.1894    0.9769
    0.0094    0.1042   -0.1180    0.9875
    0.0047    0.1047   -0.0625    0.9925
   -0.0062    0.1020    0.0488    0.9936
   -0.0205    0.0993    0.1678    0.9806
   -0.0245    0.0996    0.2014    0.9741
   -0.0311    0.1076    0.2485    0.9621
   -0.0304    0.1051    0.2683    0.9571
   -0.0320    0.1042    0.2703    0.9566
   -0.0296    0.1092    0.2278    0.9671
   -0.0170    0.0977    0.1179    0.9881
    0.0017    0.1051   -0.0361    0.9938
    0.0104    0.1037   -0.1136    0.9880
    0.0170    0.0992   -0.1892    0.9768
    0.0216    0.0984   -0.2363    0.9664
   -0.0237   -0.0562   -0.2201    0.9736
   -0.0183   -0.0573   -0.1524    0.9865
   -0.0131   -0.0553   -0.0412    0.9975
   -0.0082   -0.0518    0.0230    0.9984
   -0.0017   -0.0502    0.1028    0.9934
    0.0006   -0.0460    0.1724    0.9840
    0.0021   -0.0434    0.2328    0.9716
    0.0039   -0.0466    0.2654    0.9630
    0.0060   -0.0463    0.3012    0.9524
    0.0001   -0.0455    0.1926    0.9802
   -0.0061   -0.0541    0.0664    0.9963
   -0.0114   -0.0549   -0.0326    0.9979
   -0.0180   -0.0580   -0.1057    0.9925
   -0.0209   -0.0593   -0.1754    0.9825
   -0.0241   -0.0629   -0.2106    0.9752
   -0.0210   -0.0608   -0.2180    0.9738
   -0.0158   -0.0603   -0.0827    0.9946
    0.0031   -0.0463    0.2159    0.9753
    0.0037   -0.0468    0.2236    0.9735
    0.0022   -0.0476    0.1950    0.9796
    0.0026   -0.0482    0.2415    0.9692
    0.0045   -0.0454    0.2506    0.9670
    0.0038   -0.0460    0.2479    0.9677
    0.0030   -0.0487    0.2556    0.9655
    0.0005   -0.0487    0.1781    0.9828
];



