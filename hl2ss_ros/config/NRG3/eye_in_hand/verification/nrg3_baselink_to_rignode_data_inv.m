%% Eye-In-Hand Calibration Data

% Data
function [q_headset2world, q_target2cam,t_headset2world,t_target2cam ]= nrg3_baselink_to_rignode_data_inv()

% Position data from world to holoLens/base_link
t_headset2world=[
    0.1402   -0.4976    0.4337
   -0.2565   -0.6156    0.4344
   -0.9056   -1.4590    0.4235
   -1.0350   -1.8581    0.4174
    2.0450    0.2733    0.4756
    0.9139    1.4289    0.4603
    0.4211    1.5769    0.4461
   -0.8852    1.6269    0.4242
   -1.8420    1.1748    0.4012
    0.6969    1.6270    0.4554
    0.6417   -1.6975    0.4484
   -1.1300   -1.6470    0.4132
   -1.4442   -2.0670    0.4027
    2.1412   -0.4630    0.4759
    0.9923    1.3024    0.4589
    1.3228   -0.9120    0.4581
    1.2253   -1.4340    0.4526
    0.2290    2.0782    0.4454
   -1.5684    2.1879    0.4029
   -2.4684    1.4433    0.3749
];

% Rotation data from world to holoLens/base_link
q_headset2world=[
   -0.0024   -0.0041    0.9953    0.0971
   -0.0037   -0.0029    0.9342    0.3568
    0.0084   -0.0001   -0.9538    0.3003
    0.0096   -0.0017   -0.9325    0.3611
    0.0006   -0.0087   -0.0537    0.9985
   -0.0002   -0.0100    0.0991    0.9950
   -0.0016   -0.0087    0.1601    0.9871
   -0.0049   -0.0069    0.5590    0.8291
   -0.0079   -0.0048    0.7490    0.6625
   -0.0004   -0.0089    0.1767    0.9842
    0.0088   -0.0051   -0.7785    0.6276
    0.0100   -0.0016   -0.9440    0.3297
    0.0104   -0.0022   -0.9194    0.3931
    0.0018   -0.0079   -0.2479    0.9687
    0.0002   -0.0083    0.0857    0.9963
    0.0052   -0.0057   -0.5653    0.8249
    0.0059   -0.0046   -0.5888    0.8082
   -0.0031   -0.0086    0.3347    0.9423
   -0.0069   -0.0058    0.6021    0.7984
   -0.0109   -0.0033    0.7496    0.6618
];

% Position data from hololens/rm_vlc_leftfront to april_tag
t_target2cam=[
    0.0235   -0.5543    0.3931
   -0.3742   -0.6611    0.3492
   -1.0396   -1.4860    0.2527
   -1.1782   -1.8809    0.2249
    1.9319    0.1620    0.6543
    0.8396    1.3489    0.5408
    0.3544    1.5113    0.4765
   -0.9396    1.5979    0.3165
   -1.8993    1.1737    0.1818
    0.6297    1.5531    0.5160
    0.4888   -1.7649    0.4371
   -1.2668   -1.6669    0.2141
   -1.5886   -2.0787    0.1620
    2.0083   -0.5763    0.6505
    0.9167    1.2221    0.5458
    1.1867   -1.0021    0.5368
    1.0757   -1.5212    0.5099
    0.1776    2.0178    0.4653
   -1.6013    2.1780    0.2324
   -2.5124    1.4593    0.0943
];

% Rotation data from hololens/rm_vlc_leftfront to april_tag
q_target2cam=[
   -0.0551   -0.0185    0.9922    0.1102
   -0.0506   -0.0306    0.9276    0.3688
    0.0624   -0.0077   -0.9559    0.2868
    0.0630   -0.0127   -0.9354    0.3477
    0.0125   -0.0618   -0.0672    0.9957
    0.0035   -0.0644    0.0854    0.9943
   -0.0012   -0.0632    0.1463    0.9872
   -0.0274   -0.0565    0.5466    0.8350
   -0.0421   -0.0472    0.7383    0.6715
   -0.0011   -0.0634    0.1629    0.9846
    0.0564   -0.0318   -0.7853    0.6157
    0.0638   -0.0108   -0.9465    0.3162
    0.0633   -0.0150   -0.9227    0.3799
    0.0239   -0.0577   -0.2606    0.9634
    0.0046   -0.0627    0.0720    0.9954
    0.0431   -0.0450   -0.5753    0.8156
    0.0449   -0.0428   -0.5985    0.7987
   -0.0126   -0.0623    0.3212    0.9449
   -0.0321   -0.0540    0.5899    0.8050
   -0.0451   -0.0456    0.7389    0.6708
];



