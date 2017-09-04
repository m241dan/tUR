#!/home/korisd/torch/install/bin/th

require 'gnuplot'

gnuplot.figure(1)
gnuplot.title( 'Test' )
gnuplot.xlabel( 'X' )
gnuplot.ylabel( 'Y' )
x_one=torch.Tensor( {1, 2, 3, 4, 5, 6, 7, 8, 9, 10 } )
y_one=torch.rand(10)
x_two=torch.rand(10)
y_two=torch.rand(10)

gnuplot.plot( { "ONE", x_one, y_one }, { "TWO", x_two, y_two } )
