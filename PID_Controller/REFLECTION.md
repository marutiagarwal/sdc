I finetuned the parameters manually.

## Effect of P components 
P accounts for present values of the error. For example, if the error is large and positive, the control output will also be large and positive. The optimized value of P used is : 0.5

<table>
	<th>Param state</th><th>Description</th>
	<tr>
		<td>Higher P,
		P = 1.0</td>
		<td>Sharp turns and flickers a lot. Made to a very little distance from the beginning. Went off track before the bridge.</td>
	</tr>
	<tr>
		<td>Lower P, P = 0.001</td>
		<td>Went off the track at the beginning itself and count not come back on track. </td>
	</tr>
</table>

## Effect of I components 
I accounts for past values of the error. For example, if the current output is not sufficiently strong, the integral of the error will accumulate over time, and the controller will respond by applying a stronger action. The optimized value of I used is : 0.0001

<table>
	<th>Param state</th><th>Description</th>
	<tr>
		<td>Higher I,
		I = 0.1</td>
		<td>Was taking sharp turns and crossed the bridge. Went off the track at the very first steep turn after the bridge and could not come back. </td>
	</tr>
	<tr>
		<td>Lower I, I = 0.00001</td>
		<td>Quite stable drive. Completed the track. Can easily be chosen as an acceptable value.</td>
	</tr>
</table>

## Effect of D components 
D accounts for possible future trends of the error, based on its current rate of change. For example, continuing the P example above, when the large positive control output succeeds in bringing the error closer to zero, it also puts the process on a path to large negative error in the near future; in this case, the derivative turns negative and the D module reduces the strength of the action to prevent this overshot. The optimized value of D used is : 5.0

<table>
	<th>Param state</th><th>Description</th>
	<tr>
		<td>Higher D,
		D = 0.1</td>
		<td>Sharp turns and flickers a lot. Made to a very little distance from the beginning. Went off track and count not return to the track.</td>
	</tr>
	<tr>
		<td>Lower D, D = 100</td>
		<td>Slow and stable drive. Completes the entire track. Though, speed is much slower than our best model.</td>
	</tr>
</table>