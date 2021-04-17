
% Directory /Users/mosesnah/Documents/projects/whip-project-targeting/MuJoCo/results/20210415_172707/data_log.txt

[ dist, idx ] = sort( output.output, 'descend' );

wholeDist     = rawData.dist;
wholeTime     = rawData.currentTime;
wholeGeomXYZ  = rawData.geomXYZPositions;
wholeJointPos = rawData.jointPositions;

wholeDist     = reshape(     wholeDist ,     460, 150 )';
wholeTime     = reshape(     wholeTime ,     460, 150 )';
wholeGeomXYZ  = reshape(  wholeGeomXYZ , 87, 460, 150 );
wholeJointPos = reshape( wholeJointPos , 54, 460, 150 );

for i = 1 : 150
    
    data.tVec     =  wholeTime( idx( i ), : );
    data.XYZPos   =  squeeze( wholeGeomXYZ( :, :, idx( i ) ) );
    data.jointPos =  squeeze( wholeJointPos( :, :, idx( i ) ) );
    data.dist     =  wholeDist( idx( i ), : );
    data.minout   =  dist(i);
    wholeData( i ) = data;
    
end

% save( 'forMahdi.mat', 'wholeData' )
hold on 
for i = 1 : 29
    
    plot3( wholeData(150).XYZPos( 3 * i - 2, : ), wholeData(150).XYZPos( 3 * i - 1, : ), wholeData(150).XYZPos( 3 * i , : ) )
    
end

%%
for i = 1 : 150
    
    
    tmp = reshape( wholeData(i).XYZPos, 3, 29, 460 );
    wholeData( i ).XYZPos = tmp( :, 2:end, : );
    
end