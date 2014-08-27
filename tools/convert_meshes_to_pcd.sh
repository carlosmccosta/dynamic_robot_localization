#!/bin/sh

##################################################
################### parameters ###################
##################################################

search_path=${1:-'./'}
search_extension=${2:-'.stl'}
max_parallel_jobs=${3:-4}
conversion_extension=${4:-'.ply'}
conversion_directory=${5:-'pointclouds'}
meshlab_filters=${6:-'meshlab_filters.mlx'}
meshlab_filters_options=${7:-'-om vq vn'} #-om vc vq vn
final_extension=${8:-'.pcd'}



echo "####################################################################################################"
echo "##### Converting meshes in directory ${search_path} with ${search_extension} extension"
echo "####################################################################################################\n"


######### create output directory
mkdir -p "${search_path}/${conversion_directory}"


######### convert files
export job_ids=''
export jobs_running=0


terminate_jobs() {
	running_job_ids=`jobs -p`
	echo " --> Killing job ids ${running_job_ids}"
	for job_id in ${running_job_ids}
	do
		echo " !> Killing job with id ${job_id}"
		kill ${job_id}
	done
	exit 0
}

wait_for_jobs() {
	running_job_ids=`jobs -p`
	echo "\nWaiting for jobs to finish (ids: ${running_job_ids})...\n"
	for job_id in ${running_job_ids}
	do
		wait
	done
}

trap terminate_jobs 1 2 3 9 15

find "${search_path}" -maxdepth 1 -name "*${search_extension}" -type f -printf "%f\n" | while read file; do
	while [ ${jobs_running} -ge ${max_parallel_jobs} ]
	do
		echo " -> Reached maximum number of parallel jobs allowed (${max_parallel_jobs}). Waiting..."
		wait
		jobs_running=`expr ${jobs_running} - 1`
	done
	
	{
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
	} &
	
	job_ids="${job_ids} $!"
	jobs_running=`expr ${jobs_running} + 1`
	echo "===> job_ids: ${job_ids}"
	echo "===> jobs_running: ${jobs_running}"
	test="bbb"
done

wait_for_jobs

echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Finished mesh conversion. Converted files in directory ${search_path}/${conversion_directory}"
echo "----------------------------------------------------------------------------------------------------"
