
import pickle as pkl 
import numpy as np 
import pandas as pd
from sklearn.svm import LinearSVC
from sklearn.model_selection import GridSearchCV 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 

df = pd.read_csv('redorange_values.csv') 


# visualize 
cmap = {
    'MY_BLUE':'blue',
    'MY_GREEN':'green',
}
apply_cmap = lambda c: cmap[c]
vec_apply_cmap = np.vectorize(apply_cmap) 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(df['R'], df['G'], df['B'], c=vec_apply_cmap(df['COLOUR']).tolist()) 
plt.savefig("colour_values.png") 
plt.show()




# grid search and validate accuracy first
param_grid = {
    'C': [0.1, 0.5, 1, 5, 10],
}
gscv = GridSearchCV(LinearSVC(class_weight='balanced'), param_grid=param_grid, verbose=0)
gscv.fit(df[['R', 'G', 'B']], df['COLOUR'])

print("RESULTS:", gscv.cv_results_)

param_idx = gscv.cv_results_['rank_test_score'].argmin()
best_c = gscv.cv_results_['param_C'][param_idx] 
print("BEST C:", best_c) 



# get final model and gen c++ code 
model = LinearSVC(C=best_c, class_weight='balanced') 
model.fit(df[['R', 'G', 'B']], df['COLOUR'])


num_classes = len(model.classes_) 
def predict(r, g, b):
    if num_classes == 2:
        # Binary classification: only one decision boundary
        score = model.intercept_[0]
        for cidx, c in enumerate([r, g, b]):
            score += model.coef_[0][cidx] * c
        # Return class 1 if score > 0, else class 0
        return [score, -score]  # Convert to 2-class scores for consistency
    else:
        # Multi-class classification
        allvs = [0 for _ in range(num_classes)]
        for cidx, c in enumerate([r, g, b]):
            for i in range(num_classes):
                allvs[i] += model.coef_[i][cidx] * c
        
        for i in range(num_classes):
            allvs[i] += model.intercept_[i]
        
        return allvs
# Sanity check
if num_classes == 2:
    scores = np.dot(df[['R', 'G', 'B']].values[[0]], model.coef_.T) + model.intercept_
    predfn = predict(*df[['R', 'G', 'B']].values[0].tolist())
    print("SANITY CHECK:")
    print(f"Model score: {scores[0][0]}")
    print(f"Predict fn: {predfn[0]}")
else:
    scores = np.dot(df[['R', 'G', 'B']].values[[0]], model.coef_.T) + model.intercept_
    predfn = predict(*df[['R', 'G', 'B']].values[0].tolist())
    print("SANITY CHECK:") 
    print(scores.ravel()-predfn) 


# now output c++ code


def format_2d_list_to_cpp(data):#, c_type="int", name="myArray"):

    if not data:
        return f"{c_type} {name}[0][0] = {{}};"

    rows = len(data)
    cols = len(data[0])

    # Format each inner list as a C++ inner initializer list {val1, val2, ...}
    inner_rows = []
    for row in data:
        formatted_row = ', '.join(map(str, row))
        inner_rows.append(f"{{{formatted_row}}}")

    # Combine inner lists into the full C++ initializer format {{...}, {...}, ...}
    body = ', '.join(inner_rows)
    return '{'+body+'}' 
    
    # Determine the appropriate C++ declaration string
    declaration_str = f"{c_type} {name}[{rows}][{cols}]"
    
    return f"{declaration_str} = {{\n    {body}\n}};"

colcoef_str = format_2d_list_to_cpp(model.coef_.tolist())

colintc_inner_str = ", ".join(map(str, model.intercept_.tolist()))
colintc_str = f"{{ {colintc_inner_str} }}"

# str(model.coef_.tolist()) 
# str(model.intercept_.tolist())
if num_classes == 2:
    # Binary classification - only one set of coefficients
    colcoef_str = '{' + ', '.join(map(str, model.coef_[0].tolist())) + '}'
    colintc_str = str(model.intercept_[0])
    
    print("CODE STARTS HERE")
    print()
    print('''
float col_coef[3] = '''+colcoef_str+''';
float col_intc = '''+colintc_str+''';
int col_predict(float r, float g, float b) {
    float score = col_intc;
    float rgb[3] = {r, g, b};
    
    for (int i=0; i<3; i++) {
        score += col_coef[i] * rgb[i];
    }
    
    // Return 1 if score > 0, else 0
    return (score > 0) ? 1 : 0;
}

enum { '''+', '.join([a.strip() for a in model.classes_])+''' };
''')
else:
    # Multi-class case (your existing code)
    colcoef_str = format_2d_list_to_cpp(model.coef_.tolist())
    print("CODE STARTS HERE")
    print() 
    print('''
    int col_nc = '''+str(num_classes)+'''; 
    float col_coef['''+str(num_classes)+'''][3] = '''+colcoef_str+''';
    float col_intc['''+str(num_classes)+'''] = '''+colintc_str+''';
    int col_predict(float r, float g, float b) {
        float allvs['''+str(num_classes)+'''];
        for (int i=0; i<col_nc; i++) {
            allvs[i] = col_intc[i];
        }

        float c; 
        for (int cidx=0; cidx<3; cidx++) {
            if (cidx==0) c=r;
            else if (cidx==1) c=g;
            else c=b;
            for (int i=0; i<col_nc; i++){
                allvs[i] += col_coef[i][cidx]*c;
            }
        }

        // get index of max
        int maxidx = 0; int maxsofar = allvs[0];
        for (int i=1; i<col_nc; i++) {
            if (allvs[i] > maxsofar) {
                maxidx = i;
                maxsofar = allvs[i];
            }
        }
        return maxidx;
    }

enum { '''+',\n   '.join([a.strip() for a in model.classes_])+''' };
''')#.format(nc=num_classes, coef=model.coef_, intc=model.intercept_))

print() 
print("NOTE: THE COLOURS ARE AS FOLLOWS:")
print(model.classes_)

# save model
with open('svm_model_blue_green.pkl', 'wb') as fout:
    pkl.dump(model, fout) 
