function calibration = parforInit(total_num_dataset, opts)
    calibration(total_num_dataset) = struct();
    calibration(total_num_dataset).H_SR = [];
    calibration(total_num_dataset).H_SNR = [];
    calibration(total_num_dataset).H_NSR = [];
    calibration(total_num_dataset).H_NSNR = [];

    calibration(total_num_dataset).P_SR = [];
    calibration(total_num_dataset).P_SNR = [];
    calibration(total_num_dataset).P_NSR = [];
    calibration(total_num_dataset).P_NSNR = [];

    calibration(total_num_dataset).RMSE_SR = [];
    calibration(total_num_dataset).RMSE_SNR = [];
    calibration(total_num_dataset).RMSE_NSR = [];
    calibration(total_num_dataset).RMSE_NSNR = [];

    calibration(total_num_dataset).All = [];
    calibration(total_num_dataset).All.SR = [];
    calibration(total_num_dataset).All.SNR = [];
    calibration(total_num_dataset).All.NSR = [];
    calibration(total_num_dataset).All.NSNR = [];
    
    calibration(total_num_dataset).count.training.SNR = [];
    calibration(total_num_dataset).count.training.SR = [];
    calibration(total_num_dataset).count.training.NSR = [];
    calibration(total_num_dataset).count.training.NSNR = [];

    calibration(total_num_dataset).count.validation.SNR = [];
    calibration(total_num_dataset).count.validation.SR = [];
    calibration(total_num_dataset).count.validation.NSR = [];
    calibration(total_num_dataset).count.validation.NSNR = [];


    calibration(total_num_dataset).error_struc = [];
    calibration(total_num_dataset).error_struc.training_results.id = []';
    calibration(total_num_dataset).error_struc.training_results.name = [];
    calibration(total_num_dataset).error_struc.training_results.NSNR_RMSE = [];
    calibration(total_num_dataset).error_struc.training_results.NSR_RMSE = [];
    calibration(total_num_dataset).error_struc.training_results.SNR_RMSE = [];
    calibration(total_num_dataset).error_struc.training_results.SR_RMSE = [];


    calibration(total_num_dataset).error_struc.training(opts.num_training).id = [];   
    calibration(total_num_dataset).error_struc.training(opts.num_training).name = [];
    calibration(total_num_dataset).error_struc.training(opts.num_training).NSNR_RMSE = [];
    calibration(total_num_dataset).error_struc.training(opts.num_training).NSR_RMSE = [];
    calibration(total_num_dataset).error_struc.training(opts.num_training).SNR_RMSE = [];
    calibration(total_num_dataset).error_struc.training(opts.num_training).SR_RMSE = [];

    calibration(total_num_dataset).error_struc.validation(opts.num_validation).id = [];   
    calibration(total_num_dataset).error_struc.validation(opts.num_validation).name = [];
    calibration(total_num_dataset).error_struc.validation(opts.num_validation).NSNR_RMSE = [];
    calibration(total_num_dataset).error_struc.validation(opts.num_validation).NSR_RMSE = [];
    calibration(total_num_dataset).error_struc.validation(opts.num_validation).SNR_RMSE = [];
    calibration(total_num_dataset).error_struc.validation(opts.num_validation).SR_RMSE = [];
end