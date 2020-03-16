top = [
2.3178	0.6207
2.2559	0.5926
];

non_top = [
2.0256	0.3907
2.3247	0.3946
];

% using refinement outperforms xx%
refinement_consistency = (top(1,:) - top(2,:)) ./ top(1,:) * 100;
refinement_none_consistency = (non_top(1, :) - non_top(2,:)) ./ non_top(1,:) * 100;
refinement_consistency'
refinement_none_consistency'


% using top-consistency outperforms xx%
SNR_consistency = (top(1,:) - non_top(1,:)) ./ non_top(1,:) * 100;
SR_consistency = (top(2,:) - non_top(2,:)) ./ non_top(2,:) * 100;
SNR_consistency'
SR_consistency'


%% two tags
clc, clear
two_tags_S1 = [
8.7363	4.4712	7.3851	4.1269	7.1884	11.9767 
2.3303	2.3230	1.6318	1.3694	1.9637	2.7427
];

two_tags_S2 = [
3.3213	4.9169	5.2951	4.0811	4.4345	7.7397
2.1368	3.2027	3.2589	2.4480	4.1563	4.4254
];

two_tags_S3 = [
4.9909	9.5620	8.7533	4.6421	10.1308	15.7764
4.6720	5.9350	6.9331	4.4352	8.9493	15.3862
];

two_tags_S4 = [
21.2271	22.1641	17.5779	15.9909	8.8797	15.3636
4.3681	4.7176	4.4804	3.9004	3.3891	3.7440
];

two_tags_S5 = [
3.4621	8.3131	4.8500	7.6217	7.4838	12.4364
2.0590	2.9541	3.0224	3.5483	3.1386	6.0885
];

two_tags_S6 = [
29.4400	27.5404	27.9955	9.7005	20.6511	9.5050
5.1207	5.0574	5.3537	2.0539	4.1739	2.4194
];

two_tags_S7 = [
7.7991	9.9647	7.6857	4.1640	6.1619	2.3398
2.1563	2.7837	3.1058	1.3234	2.4537	2.0838
];

two_tags_all = [
two_tags_S1, two_tags_S2, two_tags_S3, two_tags_S4, two_tags_S5, two_tags_S6, two_tags_S7
];

disp("------------ 2 tags")
mean_two_tags_all = mean(two_tags_all')'
std_two_tags_all = std(two_tags_all')'
two_tags_mean_outperform_ = (mean_two_tags_all(1) - mean_two_tags_all(2))/mean_two_tags_all(1)
two_tags_std_outperform_ = (std_two_tags_all(1) - std_two_tags_all(2))/std_two_tags_all(1)

%% four tags
four_tags_S67 = [
6.9426	9.6281	6.9136	3.9650	5.6420
2.2409	2.5607	2.9773	1.8215	2.1995
];
four_tags_S46 = [
4.2476	8.5054	4.6712	4.3311	2.8748
1.0746	1.8871	2.3619	1.5350	2.6191
];

four_tags_S56 = [
2.9309	8.0801	4.1196	3.4985	3.2204
1.1541	2.1026	2.5017	1.4267	2.5693
];

four_tags_S25 = [
3.1991	4.7311	5.4558	4.8344	8.6576
0.9538	2.3810	1.4556	1.1592	2.5277
];

four_tags_S23 = [
2.8820	4.4672	3.9536	3.7802	6.2324
1.0896	1.5349	1.5938	1.5597	2.7543
];

four_tags_S17 = [
8.2471	4.3297	3.7712	4.1023	2.4679
1.9973	2.3987	1.5926	1.4978	1.6130
];

four_tags_S16 = [
8.0573	4.1762	3.6859	3.8973	3.2569
1.9154	2.3361	1.2906	1.2827	2.3205
];

four_tags_ALL = [
four_tags_S67, four_tags_S46 four_tags_S56  four_tags_S25 four_tags_S23 four_tags_S17 four_tags_S16
];

disp("------------ 4 tags")
mean_four_tags_all = mean(four_tags_ALL')'
std_four_tags_all = std(four_tags_ALL')'
two_tags_mean_outperform_ = (mean_four_tags_all(1) - mean_four_tags_all(2))/mean_four_tags_all(1)
two_tags_std_outperform_ = (std_four_tags_all(1) - std_four_tags_all(2))/std_four_tags_all(1)

%% 6 tags
six_tags_S456 = [
2.9824	8.0943	4.1273	3.0841
0.9713	1.9887	2.4395	2.4369    
];

six_tags_S235 = [
2.7816	4.5817	3.8262	6.6229
1.0022	1.4755	1.1722	2.7791
];

six_tags_S137 = [
8.1677	3.6688	3.9558	2.2913
1.9994	1.5854	1.4754	1.5124 
];

six_tags_S234 = [
2.9428	3.8913	2.4828	3.7673
0.9523	1.5275	1.3631	2.4347    
];

six_tags_S123 = [
4.5086	3.8577	3.7165	6.4413
1.3698	1.4807	1.3422	2.3156    
];


six_tags_S567 = [
3.2743	8.2067	4.2628	3.5584
0.9594	2.0149	2.3877	1.3754    
];

six_tags_ALL = [
six_tags_S456, six_tags_S235 six_tags_S137  six_tags_S234 six_tags_S123 six_tags_S567 
];
disp("------------ 6 tags")

mean_six_tags_all = mean(six_tags_ALL')'
std_six_tags_all = std(six_tags_ALL')'
two_tags_mean_outperform_ = (mean_six_tags_all(1) - mean_six_tags_all(2))/mean_six_tags_all(1)
two_tags_std_outperform_ = (std_six_tags_all(1) - std_six_tags_all(2))/std_six_tags_all(1)


%% eight tags

eight_tags_S1357 = [
8.1362	3.4616	2.2642
1.9886	1.4743	1.4136
];

eight_tags_S1257 = [
4.2049	3.6159	2.2928
2.4070	1.4788	1.4650
];

eight_tags_S1345 = [
8.0593	2.4532	3.8397
2.0563	1.2256	2.6797
];

eight_tags_S2457 = [
3.0803	4.1893	2.3307
0.9530	2.4280	1.4865
];


eight_tags_ALL = [
eight_tags_S1357 eight_tags_S1257  eight_tags_S1345  eight_tags_S2457
];

disp("------------ 8 tags")
mean_eight_tags_all = mean(eight_tags_ALL')'
std_eight_tags_all = std(eight_tags_ALL')'
two_tags_mean_outperform_ = (mean_eight_tags_all(1) - mean_eight_tags_all(2))/mean_eight_tags_all(1)
two_tags_std_outperform_ = (std_eight_tags_all(1) - std_eight_tags_all(2))/std_eight_tags_all(1)



%%
all_data = [
two_tags_all, four_tags_ALL, six_tags_ALL, eight_tags_ALL    
];
mean_all = mean(all_data')'
std_all = std(all_data')'
