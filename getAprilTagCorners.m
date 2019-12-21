%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
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

function AprilTagCorners = getAprilTagCorners(bag_file)
    % XXX: HACK!! need to be changed

    % build data base
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 3Tags-OldLiDAR.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    AprilTagCornerData(1).name = "3Tags-OldLiDAR.bag";
    AprilTagCornerData(1).small = [394, 349, 413, 367;
                                 249, 267, 294, 312;
                                 1, 1, 1, 1];
    AprilTagCornerData(1).large = [526, 447, 569, 490;
                                261, 297, 341, 379;
                                1, 1, 1, 1];                         

    AprilTagCornerData(1).medium = [269, 237, 284, 251;
                                      296, 313, 326, 343;
                                     1, 1, 1, 1];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab2-closer.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(2).name = "lab2-closer.bag";
    AprilTagCornerData(2).small = [197, 153, 220, 176;
                                 250, 273, 292, 315;
                                 1, 1, 1, 1];
    AprilTagCornerData(2).large = [340, 263, 406, 316;
                                236, 313, 341, 417;
                                1, 1, 1, 1];    
    AprilTagCornerData(2).medium = [];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab_angled.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(3).name = "lab_angled.bag";
    AprilTagCornerData(3).small = [];
    AprilTagCornerData(3).large = [340, 263, 406, 316;
                                    236, 313, 341, 417;
                                    1, 1, 1, 1];    
    AprilTagCornerData(3).medium = [];            

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab3-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(4).name = "lab3-closer-cleaner.bag";
    AprilTagCornerData(4).small = [200, 157, 223, 180;
                                    251, 275, 292, 315;
                                    1, 1, 1, 1];
    AprilTagCornerData(4).large = [333, 248, 418, 328;
                                   239, 322, 328, 416;
                                   1, 1, 1, 1];    
    AprilTagCornerData(4).medium = [];            

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab4-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(5).name = "lab4-closer-cleaner.bag";
    AprilTagCornerData(5).small = [250, 206, 272, 230;
                                   257, 281, 299, 323;
                                   1, 1, 1, 1];
    AprilTagCornerData(5).large = [473, 349, 575, 435;
                                   227, 314, 361, 448;
                                   1, 1, 1, 1];    
    AprilTagCornerData(5).medium = []; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab5-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(6).name = "lab5-closer-cleaner.bag";
    AprilTagCornerData(6).small = [145, 105, 165, 127;
                                   263, 284, 299, 321;
                                   1, 1, 1, 1];
    AprilTagCornerData(6).large = [398, 281, 490, 367;
                                   233, 318, 349, 440;
                                   1, 1, 1, 1];     
    AprilTagCornerData(6).medium = []; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab6-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(7).name = "lab6-closer-cleaner.bag";
    AprilTagCornerData(7).small = [230, 191, 251, 213;
                                   260, 282, 298, 321;
                                   1, 1, 1, 1];
    AprilTagCornerData(7).large = [409, 314, 476, 372;
                                   246, 309, 349, 411;
                                   1, 1, 1, 1];    
    AprilTagCornerData(7).medium = []; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab7-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(8).name = "lab7-closer-cleaner.bag";
    AprilTagCornerData(8).small = [509, 479, 529, 498;
                                   274, 292, 305, 323;
                                   1, 1, 1, 1];
    AprilTagCornerData(8).large = [178, 79, 230, 137;
                                   253, 307, 342, 402;
                                   1, 1, 1, 1];    
    AprilTagCornerData(8).medium = []; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab8-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    AprilTagCornerData(9).name = "lab8-closer-cleaner.bag";
    AprilTagCornerData(9).small = [264, 226, 284, 246;
                                   258, 279, 296, 316;
                                   1, 1, 1, 1];
    AprilTagCornerData(9).large = [465, 372, 545, 445;
                                   222, 293, 318, 389;
                                   1, 1, 1, 1];    
    AprilTagCornerData(9).medium = [];


    AprilTagCornerData(10).name = "wavefield3-tag.bag";
    AprilTagCornerData(10).small = [517, 489, 540, 512;
                                    268, 289, 297, 319;
                                    1, 1, 1, 1];
    AprilTagCornerData(10).large = [255, 215, 282, 243;
                                    284, 313, 328, 359;
                                    1, 1, 1, 1];    
    AprilTagCornerData(10).medium = [];


    AprilTagCornerData(11).name = "wavefield5-tag.bag";
    AprilTagCornerData(11).small = [483, 443, 517, 474;
                                    236, 273, 280, 317;
                                    1, 1, 1, 1];
    AprilTagCornerData(11).large = [168, 110, 204, 152;
                                    268, 317, 329, 383;
                                    1, 1, 1, 1];    
    AprilTagCornerData(11).medium = [];

%             index = find(strcmp(List, bag_file));
    index = find(strcmp([AprilTagCornerData(:).name], bag_file));
%             AprilTagCornerData(index).name
    num_data = size(AprilTagCornerData, 2);
    for i = 1: num_data
        if i == index
            AprilTagCorners.medium = AprilTagCornerData(i).medium;
            AprilTagCorners.small = AprilTagCornerData(i).small;
            AprilTagCorners.large = AprilTagCornerData(i).large;
        end
    end
end
