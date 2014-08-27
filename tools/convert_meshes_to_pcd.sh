#!/bin/sh

##################################################
################### parameters ###################
##################################################

search_path=${1:-'./'}
search_extension=${2:-'.stl'}
conversion_extension=${3:-'.ply'}
conversion_directory=${4:-'pointclouds'}
meshlab_filters=${5:-'meshlab_filters.mlx'}
meshlab_filters_options=${6:-'-om vq vn'} #-om vc vq vn
final_extension=${7:-'.pcd'}



echo "####################################################################################################"
echo "##### Converting meshes in directory ${search_path} with ${search_extension} extension"
echo "####################################################################################################\n"


######### create output directory
mkdir -p "${search_path}/${conversion_directory}"


######### convert files
find "${search_path}" -maxdepth 1 -name "*${search_extension}" -type f -printf "%f\n" | while read file; do
	file_without_extension="${search_path}/${conversion_directory}/${file%${search_extension}}"
	stdout_log="${file_without_extension}__stdout.log"
	stderr_log="${file_without_extension}__stderr.log"
	input_file="${search_path}/${file}"
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
done


echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Finished mesh conversion. Converted files in directory ${search_path}/${conversion_directory}"
echo "----------------------------------------------------------------------------------------------------"
