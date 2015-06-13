#!/usr/bin/env bash

##################################################
################### parameters ###################
##################################################

input_file=${1:?'Must specify file to convert'}
conversion_extension=${2:-'.ply'}
meshlab_filters=${3:-'meshlab_filters.mlx'}
meshlab_filters_options=${4:-'-om vq vn'} #-om vc vq vn
final_extension=${5:-'.pcd'}



echo "####################################################################################################"
echo "##### Converting ${input_file} into ${final_extension}"
echo "####################################################################################################\n"


######### convert file
file_without_extension="${input_file%.*}"
stdout_log="${file_without_extension}__stdout.log"
stderr_log="${file_without_extension}__stderr.log"
conversion_file="${file_without_extension}${conversion_extension}"
output_file="${file_without_extension}${final_extension}"

echo "==> Filtering ${input_file} into ${conversion_file} using meshlab"
meshlabserver -i ${input_file} -o ${conversion_file} -s ${meshlab_filters} ${meshlab_filters_options} > ${stdout_log} 2> ${stderr_log}

if [ "${conversion_extension}" = '.ply' ]; then
	echo " +> Changing ply file to have curvature field instead of quality"
	sed -i 's/quality/curvature/' ${conversion_file} > ${stdout_log} 2> ${stderr_log}
fi

echo " +> Converting ${conversion_file} into ${output_file} using drl_mesh_to_pcd"
rosrun dynamic_robot_localization drl_mesh_to_pcd ${conversion_file} ${output_file} -binary 1 -compressed 1 -type auto > ${stdout_log} 2> ${stderr_log}
echo " !> Finished conversion of ${conversion_file} into ${final_extension}\n"


echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Finished mesh conversion."
echo "----------------------------------------------------------------------------------------------------"
