%{
 * Copyright (C) 2020-2030, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

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