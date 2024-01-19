using PlyIO
# using UnicodePlots
using StatsBase
using Plots

gr()

# custom functions
dropnans(v) = filter(x->!isnan(x), v)

function pivot(x, y,z, v)

  u_x = unique(x)
  u_y = unique(y) 
  u_z = unique(z) 

  V = zeros(length(u_x), length(u_y), length(u_z))
  
  for i=1:length(v)
    xi = findfirst(isequal(x[i]), u_x)
    yi = findfirst(isequal(y[i]), u_y)
    zi = findfirst(isequal(z[i]), u_z)
    V[xi, yi, zi] = v[i]
  end

  return u_x, u_y, u_z, V
end




# println(ARGS)


# @assert length(ARGS) == 2

gthr_file = "/workspaces/isaac_ros-dev/maps/run_1/clean/ros2_esdf_rotated_gridded.ply"
esdf_file = "/workspaces/isaac_ros-dev/maps/run_1/noisy/ros2_esdf_rotated_gridded.ply"
cert_file = "/workspaces/isaac_ros-dev/maps/run_1/noisy/ros2_certified_esdf_rotated_gridded.ply"

# gthr_file = "/workspaces/isaac_ros-dev/maps/run_1/clean/ros2_esdf.ply"
# esdf_file = "/workspaces/isaac_ros-dev/maps/run_1/noisy/ros2_esdf.ply"
# cert_file = "/workspaces/isaac_ros-dev/maps/run_1/noisy/ros2_certified_esdf.ply"


gthr = load_ply(gthr_file)
esdf = load_ply(esdf_file)
cert = load_ply(cert_file)


# check that the lengths are compatible
println(length(esdf["vertex"]["x"]))
println(length(cert["vertex"]["x"]))

@assert length(esdf["vertex"]["x"]) == length(cert["vertex"]["x"]) == length(gthr["vertex"]["x"])
@assert length(esdf["vertex"]["y"]) == length(cert["vertex"]["y"]) == length(gthr["vertex"]["y"])
@assert length(esdf["vertex"]["z"]) == length(cert["vertex"]["z"]) == length(gthr["vertex"]["z"])

# extract axes
x = esdf["vertex"]["x"] |> Array
y = esdf["vertex"]["y"] |> Array
z = esdf["vertex"]["z"] |> Array


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



@show sum(diff_gthr_esdf .<= -0.1) / length(diff_gthr_esdf)
@show sum(diff_gthr_cert .<= -0.1) / length(diff_gthr_cert)

# histogram(diff_gthr_esdf)
# histogram(diff_gthr_cert)


# plot heatmap
ux, uy, uz, V_gthr = pivot(x, y, z, gthr_vals)
ux, uy, uz, V_esdf = pivot(x, y, z, esdf_vals)
ux, uy, uz, V_cert = pivot(x, y, z, cert_vals)

# trim vals
function trim_vals!(x, d=0.15)
	x[x .>= d] .= 1.0
	x[x .< d] .= 0.0
end


function classify_diff(V_gt, V_es, d=0.075, d_free=0.3)

	V = 0 * (V_gt - V_es); # make sure nans are dealt with correctly

        gt_obs = V_gt .<= d
        es_free = V_es .> d_free

        V[ gt_obs .&& es_free] .= -2.0

	# V[ (V_gt .<= d) .&& (V_es .> d)] .= -2.0 # unsafely miscat
	# V[ (V_gt .> d) .&& (V_es .<= d)] .= 2.0 # safely miscat
	
	# V[  V_gt .> (V_es .+ d) ] .= 2.0 # safely miscategorized
	# V[ V_gt .< (V_es .- d) ] .= -2.0 # unsafely miscategorized
        return V


end

# trim_vals!(V_gthr)
# trim_vals!(V_esdf)
# trim_vals!(V_cert)

error_palette = palette([:red, :yellow, :green], 7)

function plot_slice(ux, uy, uz, V, at)

    zi = findfirst(isequal(at), uz)

    return Plots.heatmap(ux, uy, V[:, :, zi]'; aspect_ratio=:equal, color=error_palette, clims=(-2,2))
end







function plot_at(at)

  @show at

  p1 = plot_slice(ux, uy, uz, V_gthr, at) 
  p2 = plot_slice(ux, uy, uz, V_esdf, at) 
  p3 = plot_slice(ux, uy, uz, V_cert, at) 
  
  l = @layout [a b c]
  
  plot(p1, p2, p3, size=(1200, 400), layout=l)

end


function animate(ux, uy, uz, V; zmin=0.0, zmax=2.0)
  
  lim_z = [z for z in uz if zmin<=z<=zmax]

  anim = @animate for z in lim_z
  plot_slice(ux, uy, uz, V, z)
  Plots.title!("z=$z")
  end

  return Plots.gif(anim, "./anim.gif", fps=15)
end


function average_over_zs(ux, uy, uz, V, z_min, z_max)

zl = findfirst(z-> z > z_min, uz)
zu = findlast( z-> z < z_max, uz)

VV = zeros(length(ux), length(uy))

for i=1:length(ux)
for j = 1:length(uy)
VV[i,j] = mean(V[i,j,zl:zu])
end
end

return ux, uy, VV
end







