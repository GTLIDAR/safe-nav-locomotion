for file in *.STL;
  do meshlabserver -i $(basename $file .STL).STL -o $(basename $file .STL).obj;
done
