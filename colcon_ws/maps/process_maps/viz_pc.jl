module Viz


using MeshCat
using ColorTypes
using PlyIO
using GeometryBasics


vis = Visualizer()
open(vis)


function default_colorscheme(intensity)
    if intensity <= 0.0
        return RGB(1,0,0)
    elseif intensity <= 0.2
        return RGB(0.5, 0, 0.5)
    else
        return RGB(0, 1, 0)
    end
end


function to_pc(ply::Ply)

    pos_x = Array(ply["vertex"]["x"])
    pos_y = Array(ply["vertex"]["y"])
    pos_z = Array(ply["vertex"]["z"])
    intensity = Array(ply["vertex"]["intensity"])

    res = hcat(pos_x, pos_y, pos_z, intensity)
    
    return res
end

# allow filtering points
function to_pc(ply::Ply, filt)
    points = to_pc(ply)

    inds = filter(i-> filt(points[i, :]), 1:size(points, 1))


    return points[inds, :]

end

function viz_pc!(pc::Ply; kwargs...)
    return viz_pc!(to_pc(pc); kwargs...)
end

function viz_pc!(pc::M; color_scheme = default_colorscheme, vis=vis) where {M <: AbstractMatrix}

    verts = [Point3f( v...) for v in eachrow(pc[:, 1:3])]
    colors = color_scheme.(pc[:, 4])
    materials = [PointsMaterial(color=c, size=0.075) for c in colors]

    setobject!(vis, PointCloud(verts, colors) )

end

end


