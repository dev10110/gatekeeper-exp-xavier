using PlyIO
using UnicodePlots

dropnans(v) = filter(x->!isnan(x), v)

# println(ARGS)


# @assert length(ARGS) == 2

gthr_file = "/workspaces/isaac_ros-dev/maps/run_1/clean/ros2_esdf_rotated_gridded.ply"
esdf_file = "/workspaces/isaac_ros-dev/maps/run_1/noisy/ros2_esdf_rotated_gridded.ply"
cert_file = "/workspaces/isaac_ros-dev/maps/run_1/noisy/ros2_certified_esdf_rotated_gridded.ply"


gthr = load_ply(gthr_file)
esdf = load_ply(esdf_file)
cert = load_ply(cert_file)


# check that the lengths are compatible
println(length(esdf["vertex"]["x"]))
println(length(cert["vertex"]["x"]))

@assert length(esdf["vertex"]["x"]) == length(cert["vertex"]["x"]) == length(gthr["vertex"]["x"])


# convert to raw arrays
gthr_vals = Array(gthr["vertex"]["intensity"])
esdf_vals = Array(esdf["vertex"]["intensity"])
cert_vals = Array(cert["vertex"]["intensity"])

gthr_vals[gthr_vals .== 1000.0] .= NaN
esdf_vals[esdf_vals .== 1000.0] .= NaN 
cert_vals[cert_vals .== 1000.0] .= NaN


diff_gthr_esdf = dropnans(gthr_vals - esdf_vals)
diff_gthr_cert = dropnans(gthr_vals - cert_vals) 

@show length(diff_gthr_esdf)
@show length(diff_gthr_cert)



histogram(diff_gthr_esdf)

histogram(diff_gthr_cert)
